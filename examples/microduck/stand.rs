//! A bus master on Pollen's `rustypot` 1.8.0, reading drydock's Microduck as
//! Pollen's runtime does each tick (`duck-control`'s `bus.rs`): one fast
//! read of twelve bytes at 124 from the IMU board and the fifteen servos,
//! the IMU first. It finds every device, checks the knees count from -512,
//! turns torque on where the joints are, and checks the duck still stands
//! three seconds later, by the gravity the IMU gives, decoded as Pollen's
//! `imu.rs` decodes it. CI builds it in a project of its own, as
//! `examples/xl330/rustypot.rs`, and runs it against
//! `drydock examples/microduck/robot.yml --lockstep 50`.

use std::process::ExitCode;
use std::time::Duration;

use rustypot::servo::dynamixel::xl330::Xl330Controller;

/// The servos, in Pollen's joint order (`duck-ipc-proto`).
const IDS: [u8; 15] = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14];
const IMU: u8 = 200;

fn half(h: u16) -> f64 {
    let s = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
    let (e, m) = ((h >> 10) & 0x1F, (h & 0x3FF) as f64);
    s * if e == 0 { m * 2f64.powi(-24) } else { (1.0 + m / 1024.0) * 2f64.powi(e as i32 - 15) }
}

fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Gravity in the trunk's frame, from the board's block: its quaternion,
/// w derived, the zero3 board's mount (+90° about y) undone.
fn gravity(block: &[u8]) -> [f64; 3] {
    let [x, y, z] = [6, 8, 10].map(|i| half(u16::from_le_bytes([block[i], block[i + 1]])));
    let w = (1.0 - x * x - y * y - z * z).max(0.0).sqrt();
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let q = multiply([w, x, y, z], [s, 0.0, -s, 0.0]);
    let g = multiply(multiply([q[0], -q[1], -q[2], -q[3]], [0.0, 0.0, 0.0, -1.0]), q);
    [g[1], g[2], g[3]]
}

fn main() -> ExitCode {
    let port = std::env::args().nth(1).unwrap_or("/tmp/microduck-tty".into());
    let serial = match serialport::new(&port, 1_000_000).timeout(Duration::from_millis(10)).open() {
        Ok(s) => s,
        Err(e) => {
            eprintln!("can't open {port}: {e}");
            return ExitCode::FAILURE;
        }
    };
    let mut c = Xl330Controller::new().with_protocol_v2().with_fast_sync_read().with_serial_port(serial);
    let mut ok = true;
    let mut check = |what: &str, passed: bool| {
        println!("{} {what}", if passed { "ok  " } else { "FAIL" });
        ok &= passed;
    };
    let found = IDS.iter().filter(|&&id| c.ping(id).unwrap_or(false)).count();
    check(&format!("{found} of 15 servos answer"), found == 15);
    check("the IMU board answers", c.ping(IMU).ok() == Some(true));
    check("the knees count from -512", c.sync_read_homing_offset(&[13, 23]).ok() == Some(vec![-512, -512]));
    let Ok(here) = c.sync_read_present_position(&IDS) else {
        println!("FAIL reading the joints");
        return ExitCode::FAILURE;
    };
    let on = c.sync_write_goal_position(&IDS, &here).is_ok() && c.sync_write_torque_enable(&IDS, &[true; 15]).is_ok();
    check("torque on where the joints are", on);
    let ids: Vec<u8> = std::iter::once(IMU).chain(IDS).collect();
    let mut last = None;
    for _ in 0..150 {
        match c.sync_read_raw_data(&ids, 124, 12) {
            Ok(blocks) if blocks.len() == 16 => last = Some(gravity(&blocks[0])),
            other => {
                println!("FAIL the combined read: {other:?}");
                return ExitCode::FAILURE;
            }
        }
    }
    // Standing: gravity within about 15° of straight down.
    let g = last.unwrap_or_default();
    check(&format!("still standing: gravity [{:+.3} {:+.3} {:+.3}]", g[0], g[1], g[2]), g[2] < -0.96);
    if ok {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}
