//! The example scenarios read, against the robots they're for.

use drydock::robot::Robot;
use drydock::scenario::Scenario;

#[test]
fn the_example_scenarios_read() {
    let dir = concat!(env!("CARGO_MANIFEST_DIR"), "/examples");
    let mut read = 0;
    for robot in ["arm", "openarm"] {
        let path = format!("{dir}/{robot}");
        let joints: Vec<String> =
            Robot::load(format!("{path}/robot.yml")).unwrap().motors().map(|(_, m)| m.joint.clone()).collect();
        for entry in std::fs::read_dir(&path).unwrap() {
            let file = entry.unwrap().path();
            if file.to_string_lossy().ends_with(".scenario.yml") {
                Scenario::load(&file, &joints).unwrap_or_else(|e| panic!("{e}"));
                read += 1;
            }
        }
    }
    assert!(read >= 2, "found {read}");
}

/// The slide's motor pushing at its rated torque for a minute, with or
/// without its cooling: its status after.
fn pushed_for_a_minute(scenario: &str) -> u8 {
    use drydock::can::Frame;
    use drydock::damiao::{Feedback, Mit, Special};
    use drydock::dock::Drydock;
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/slide/robot.yml")).unwrap();
    let spec = &robot.buses[0].motors[0];
    let (limits, rated) = (spec.limits(), spec.rated_torque() as f32);
    let mut dock = Drydock::load(&robot).unwrap();
    dock.set_scenario(Scenario::parse(scenario, &["slide".to_string()]).unwrap());
    dock.deliver(0, &Frame::new(0x08, Special::Enable.encode()));
    let push = Frame::new(0x08, Mit { p: 0.0, v: 0.0, kp: 0.0, kd: 0.0, t: rated }.encode(limits));
    let mut last = None;
    for _ in 0..(60.0 / dock.timestep()) as usize {
        last = dock.deliver(0, &push);
        dock.step();
    }
    Feedback::decode(&last.unwrap().eight().unwrap(), limits).status
}

#[test]
fn a_motor_that_lost_its_cooling_overheats_at_its_rated_torque() {
    use drydock::damiao::status;
    assert_eq!(pushed_for_a_minute("faults: []"), status::ENABLED);
    assert_eq!(
        pushed_for_a_minute("faults: [{ joint: slide, fault: overheat, at: 0 s }]"),
        status::MOS_OVERTEMPERATURE
    );
}
