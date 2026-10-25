//! A bus master on Pollen's `rustypot` 1.8.0, the servo library the
//! Microduck's runtime uses, opening drydock's serial port as it opens its
//! real one: it finds the servo, sets it up, lifts the arm from hanging to
//! level with sync writes, reading back with Fast Sync Read, and checks it
//! got there. CI builds it in a project of its own (drydock doesn't depend
//! on rustypot):
//!
//! ```toml
//! [dependencies]
//! rustypot = "=1.8.0"
//! serialport = { version = "4", default-features = false }
//! ```
//!
//! Run against `drydock examples/xl330/robot.yml`.

use std::process::ExitCode;
use std::time::{Duration, Instant};

use rustypot::servo::dynamixel::xl330::Xl330Controller;

fn main() -> ExitCode {
    let port = std::env::args().nth(1).unwrap_or("/tmp/drydock-xl330".into());
    let serial = match serialport::new(&port, 1_000_000).timeout(Duration::from_millis(10)).open() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("can't open {port}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let mut c = Xl330Controller::new().with_protocol_v2().with_fast_sync_read().with_serial_port(serial);
    let checks = [
        ("servo 1 answers a ping", c.ping(1).ok() == Some(true)),
        ("servo 2 doesn't", c.ping(2).ok() == Some(false)),
        ("it's an XL330-M288", c.read_model_number(1).ok() == Some(vec![1200])),
        ("its return delay takes 0", c.write_return_delay_time(1, 0).is_ok()),
        ("its torque comes on", c.write_torque_enable(1, true).is_ok()),
    ];
    let mut ok = true;
    for (what, passed) in checks {
        println!("{} {what}", if passed { "ok  " } else { "FAIL" });
        ok &= passed;
    }
    let start = c.sync_read_present_position(&[1]).map(|p| p[0]);
    println!("hanging at {start:?} rad");
    let t0 = Instant::now();
    let mut last = f64::NAN;
    while t0.elapsed() < Duration::from_secs(1) {
        let _ = c.sync_write_goal_position(&[1], &[0.0]);
        match c.sync_read_present_position(&[1]) {
            Ok(p) => last = p[0],
            Err(e) => {
                println!("FAIL fast sync read: {e}");
                return ExitCode::FAILURE;
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    // A P loop holding 100 g at 10 cm sags a little short of level.
    let level = (0.0..0.2).contains(&last);
    println!("{} lifted to {last:+.3} rad", if level { "ok  " } else { "FAIL" });
    if ok && level {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
