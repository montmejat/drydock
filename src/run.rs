//! How time passes: with the wall clock, frames crossing each bus as long
//! as they would; or with the bus masters' cycles.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;
use std::time::{Duration, Instant};

use crate::can::timing::{Crossed, From, Rates, Timeline};
use crate::can::{wait_any, Socket};
use crate::dock::Drydock;

/// Behind the wall clock by more than this, the simulation stops trying to
/// catch up, and says so.
const MAX_LAG: Duration = Duration::from_millis(100);

/// A bus as the clock sees it: its socket, and its timing.
pub struct Line {
    pub name: String,
    pub socket: Socket,
    pub rates: Rates,
    /// From a frame reaching a motor to its answer being ready, seconds.
    pub reply: f64,
}

/// Every frame that crosses a bus, as a line of JSON: which bus, its id,
/// who sent it, its data, and when it was ready, started across and arrived,
/// in nanoseconds on the machine's monotonic clock (`CLOCK_MONOTONIC`), the
/// clock keel's traces use, so the two line up.
pub struct BusTrace {
    out: BufWriter<File>,
    /// The monotonic clock, in ns, when the run's clock read 0.
    base_ns: f64,
    flushed: f64,
}

impl BusTrace {
    pub fn create(path: &Path) -> io::Result<Self> {
        Ok(Self { out: BufWriter::new(File::create(path)?), base_ns: 0.0, flushed: 0.0 })
    }

    fn write(&mut self, bus: &str, c: &Crossed) -> io::Result<()> {
        let from = match c.from {
            From::Master => "master",
            From::Motor => "motor",
        };
        let data: String = c.frame.data[..c.frame.len as usize].iter().map(|b| format!("{b:02x}")).collect();
        let ns = |t: f64| (self.base_ns + t * 1e9).round() as u64;
        writeln!(
            self.out,
            r#"{{"bus":"{bus}","id":{},"from":"{from}","fd":{},"data":"{data}","ready_ns":{},"start_ns":{},"end_ns":{}}}"#,
            c.frame.id,
            c.frame.fd,
            ns(c.ready),
            ns(c.start),
            ns(c.end)
        )
    }

    /// Out to the file every tenth of a second: drydock ends when it's
    /// killed, and loses only that much.
    fn flush_now_and_then(&mut self, now: f64) -> io::Result<()> {
        if now - self.flushed >= 0.1 {
            self.flushed = now;
            self.out.flush()?;
        }
        Ok(())
    }
}

fn monotonic_ns() -> f64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: writes into a local.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as f64 * 1e9 + ts.tv_nsec as f64
}

/// In real time: a step of physics each timestep of the wall clock. A frame
/// from a master crosses its bus as long as it would on a real one, after
/// the frames ahead of it, and reaches the motors when it's across; their
/// answers cross it in turn, and reach the master when they're across.
///
/// With `trace`, every frame that crosses a bus goes there too.
pub fn realtime(mut dock: Drydock, lines: &[Line], mut trace: Option<BusTrace>) -> io::Result<()> {
    // SAFETY: a prctl with no pointers. Waits end within a microsecond of
    // when asked, not the default 50.
    unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, 1u64) };
    let clock = Instant::now();
    if let Some(t) = &mut trace {
        t.base_ns = monotonic_ns();
    }
    let seconds = || clock.elapsed().as_secs_f64();
    let sockets: Vec<&Socket> = lines.iter().map(|l| &l.socket).collect();
    let mut buses: Vec<Timeline> = lines.iter().map(|l| Timeline::new(l.rates)).collect();
    let dt = dock.timestep();
    let (mut start, mut steps) = (0.0, 0u64);
    loop {
        let due = start + dt * (steps + 1) as f64;
        loop {
            let now = seconds();
            for (b, bus) in buses.iter_mut().enumerate() {
                loop {
                    let arrived = bus.advance(now);
                    if arrived.is_empty() {
                        break;
                    }
                    for crossed in arrived {
                        if let Some(t) = &mut trace {
                            t.write(&lines[b].name, &crossed)?;
                        }
                        match crossed.from {
                            From::Master => {
                                if let Some(answer) = dock.deliver(b, &crossed.frame) {
                                    bus.push(answer, From::Motor, crossed.end + lines[b].reply);
                                }
                            }
                            From::Motor => lines[b].socket.send(&crossed.frame)?,
                        }
                    }
                }
            }
            if now >= due {
                break;
            }
            let wake = buses.iter().filter_map(Timeline::next_event).fold(due, f64::min);
            let ready = wait_any(&sockets, Some(Duration::from_secs_f64((wake - now).max(0.0))))?;
            for (b, _) in ready.iter().enumerate().filter(|(_, &r)| r) {
                while let Some(frame) = lines[b].socket.try_recv()? {
                    buses[b].push(frame, From::Master, seconds());
                }
            }
        }
        dock.step();
        steps += 1;
        let now = seconds();
        if let Some(t) = &mut trace {
            t.flush_now_and_then(now)?;
        }
        for (bus, line) in buses.iter_mut().zip(lines) {
            if let Some(report) = bus.report(now) {
                eprintln!("drydock: {}: {report}", line.name);
            }
        }
        let late = now - due;
        if late > MAX_LAG.as_secs_f64() {
            eprintln!(
                "drydock: {:.0} ms behind the wall clock at t = {:.3} s: catching up from here",
                late * 1e3,
                dock.time()
            );
            (start, steps) = (now, 0);
        }
    }
}

/// In lockstep with the bus masters: each frame reaches the motors at once
/// and is answered at once, with the state as it is, and `period` seconds
/// of physics run once every motor has had a frame. Time stands still while
/// the masters don't send, and frames take no time.
///
/// A master that sends a command to each motor per cycle therefore reads,
/// in the answers, the state its previous commands brought, a cycle later:
/// as on a real bus, but the same each run.
pub fn lockstep(mut dock: Drydock, lines: &[Line], period: f64) -> io::Result<()> {
    let steps = steps_per_period(dock.timestep(), period).map_err(io::Error::other)?;
    let sockets: Vec<&Socket> = lines.iter().map(|l| &l.socket).collect();
    loop {
        let ready = wait_any(&sockets, None)?;
        for (b, _) in ready.iter().enumerate().filter(|(_, &r)| r) {
            while let Some(frame) = lines[b].socket.try_recv()? {
                if let Some(answer) = dock.deliver(b, &frame) {
                    lines[b].socket.send(&answer)?;
                }
                if dock.cycle_done() {
                    for _ in 0..steps {
                        dock.step();
                    }
                }
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
