//! A bus master for a robot file's motors, as a robot's software would be:
//! it enables each motor, then swings each joint along a sine with MIT-mode
//! commands at 500 Hz, printing what the motors answer twice a second. It
//! disables them after `--seconds`, if given.
//!
//!   cargo run --example wave -- examples/arm/robot.yml [--seconds 5]

use std::error::Error;
use std::f32::consts::TAU;
use std::time::{Duration, Instant};

use drydock::can::{Frame, Socket};
use drydock::damiao::{Feedback, Mit, Special};
use drydock::robot::Robot;

const PERIOD: Duration = Duration::from_millis(2);

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let path = args.first().map_or("examples/arm/robot.yml", |s| s.as_str());
    let seconds: Option<f64> = match args.iter().position(|a| a == "--seconds") {
        Some(i) => Some(args.get(i + 1).ok_or("--seconds needs a number")?.parse()?),
        None => None,
    };
    let robot = Robot::load(path)?;
    // The first bus's motors.
    let line = &robot.buses[0];
    let bus = Socket::open(&line.can)?;
    let motors = &line.motors;
    let mut latest: Vec<Option<Feedback>> = vec![None; motors.len()];

    for m in motors {
        bus.send(&Frame::new(m.id, Special::Enable.encode()).fd(line.fd))?;
    }
    let start = Instant::now();
    for cycle in 0u64.. {
        let t = cycle as f32 * PERIOD.as_secs_f32();
        if seconds.is_some_and(|s| t as f64 >= s) {
            break;
        }
        for (i, m) in motors.iter().enumerate() {
            // Each joint at its own pace, within half a radian of level.
            let w = TAU * 0.25 * (i + 1) as f32;
            let c = Mit { p: 0.5 * (w * t).sin(), v: 0.5 * w * (w * t).cos(), kp: 40.0, kd: 1.0, t: 0.0 };
            bus.send(&Frame::new(m.id, c.encode(m.limits())).fd(line.fd))?;
        }
        let due = start + PERIOD * (cycle as u32 + 1);
        while let Some(wait) = due.checked_duration_since(Instant::now()) {
            let Some(frame) = bus.recv(Some(wait))? else { continue };
            let i = motors.iter().position(|m| m.master_id == frame.id);
            if let (Some(i), Some(data)) = (i, frame.eight()) {
                latest[i] = Some(Feedback::decode(&data, motors[i].limits()));
            }
        }
        if cycle % 250 == 0 {
            print!("t = {t:5.1} s");
            for (m, f) in motors.iter().zip(&latest) {
                match f {
                    Some(f) => print!("  {}: {:+.3} rad {:+.2} N m [{:X}]", m.joint, f.p, f.t, f.status),
                    None => print!("  {}: silent", m.joint),
                }
            }
            println!();
        }
    }
    for m in motors {
        bus.send(&Frame::new(m.id, Special::Disable.encode()).fd(line.fd))?;
    }
    Ok(())
}
