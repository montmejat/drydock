//! How time passes: with the wall clock, or with the bus master's cycles.

use std::io;
use std::time::{Duration, Instant};

use crate::can::Bus;
use crate::dock::Drydock;

/// Behind the wall clock by more than this, the simulation stops trying to
/// catch up, and says so.
const MAX_LAG: Duration = Duration::from_millis(100);

/// In real time: a step of physics each timestep of the wall clock, and
/// frames answered as they come, with the state of the moment.
pub fn realtime(mut dock: Drydock, bus: &Bus) -> io::Result<()> {
    // SAFETY: a prctl with no pointers. Waits end within a microsecond of
    // when asked, not the default 50.
    unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, 1u64) };
    let dt = Duration::from_secs_f64(dock.timestep());
    let mut start = Instant::now();
    let mut steps: u32 = 0;
    loop {
        let due = start + dt * (steps + 1);
        loop {
            let now = Instant::now();
            if now >= due {
                break;
            }
            if let Some(frame) = bus.recv(Some(due - now))? {
                if let Some(reply) = dock.handle(&frame) {
                    bus.send(&reply)?;
                }
            }
        }
        dock.step();
        steps += 1;
        let late = Instant::now().saturating_duration_since(due);
        if late > MAX_LAG {
            eprintln!(
                "drydock: {} ms behind the wall clock at t = {:.3} s: catching up from here",
                late.as_millis(),
                dock.time()
            );
            start = Instant::now();
            steps = 0;
        } else if steps == u32::MAX {
            start = due;
            steps = 0;
        }
    }
}

/// In lockstep with the bus master: each frame answered at once, with the
/// state as it is, and `period` seconds of physics once every motor has had
/// a frame. Time stands still while the master doesn't send.
///
/// A master that sends a command to each motor per cycle therefore reads,
/// in the answers, the state its previous commands brought, a cycle later:
/// as on a real bus, but the same each run.
pub fn lockstep(mut dock: Drydock, bus: &Bus, period: f64) -> io::Result<()> {
    let steps = steps_per_period(dock.timestep(), period).map_err(io::Error::other)?;
    loop {
        let Some(frame) = bus.recv(None)? else { continue };
        if let Some(reply) = dock.handle(&frame) {
            bus.send(&reply)?;
        }
        if dock.cycle_done() {
            for _ in 0..steps {
                dock.step();
            }
        }
    }
}

/// How many timesteps make a period, which must be a whole number of them.
pub fn steps_per_period(timestep: f64, period: f64) -> Result<u32, String> {
    let n = (period / timestep).round();
    if n < 1.0 || ((n * timestep - period) / period).abs() > 1e-9 {
        return Err(format!(
            "a lockstep period of {} ms isn't a whole number of the scene's {} ms timesteps",
            period * 1e3,
            timestep * 1e3
        ));
    }
    Ok(n as u32)
}
