//! A motor driving a slide joint through a transmission.

use drydock::can::Frame;
use drydock::damiao::{status, Feedback, Mit, Special};
use drydock::dock::Drydock;
use drydock::robot::Robot;

#[test]
fn a_motor_holds_a_weight_through_its_transmission() {
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/slide/robot.yml")).unwrap();
    let limits = robot.buses[0].motors[0].limits();
    let mut dock = Drydock::load(&robot).unwrap();
    // The joint starts at 0: the motor, at (0 - 0.044) / 0.042 rad.
    let f = dock.deliver(0, &Frame::new(0x08, Special::Enable.encode())).unwrap();
    let f = Feedback::decode(&f.eight().unwrap(), limits);
    assert!((f.p as f64 + 0.044 / 0.042).abs() < 1e-3, "{f:?}");

    // To -0.5 rad: the joint to 0.044 - 0.021 = 0.023 m.
    let hold = Mit { p: -0.5, v: 0.0, kp: 200.0, kd: 2.0, t: 0.0 }.encode(limits);
    let mut answer = None;
    for _ in 0..4000 {
        answer = dock.deliver(0, &Frame::new(0x08, hold));
        dock.step();
    }
    let f = Feedback::decode(&answer.unwrap().eight().unwrap(), limits);
    assert_eq!(f.status, status::ENABLED);
    assert!((f.p + 0.5).abs() < 0.01, "{f:?}");
    // A kilogram's weight on the slide is 9.81 N: 9.81 × 0.042 N m at the motor.
    assert!((f.t as f64 - 9.81 * 0.042).abs() < 0.02, "{f:?}");
}
