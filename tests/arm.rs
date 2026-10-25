//! The example arm in MuJoCo, frame by frame, without a bus.

use drydock::can::Frame;
use drydock::damiao::{reg, status, Feedback, Mit, Register, RegisterReply, Special};
use drydock::dock::Drydock;
use drydock::robot::Robot;
use drydock::run::steps_per_period;
use drydock::scenario::{Change, Scenario};

const SHOULDER: u16 = 0x01;
const ELBOW: u16 = 0x02;

fn arm() -> (Drydock, Robot) {
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/arm/robot.yml")).unwrap();
    (Drydock::load(&robot).unwrap(), robot)
}

/// The arm, with what a scenario's text says happens to it.
fn arm_with(scenario: &str) -> (Drydock, Robot) {
    let (mut dock, robot) = arm();
    let joints: Vec<String> = robot.motors().map(|(_, m)| m.joint.clone()).collect();
    dock.set_scenario(Scenario::parse(scenario, &joints).unwrap());
    (dock, robot)
}

fn feedback(robot: &Robot, frame: Frame) -> Feedback {
    let (_, m) = robot.motors().find(|(_, m)| m.master_id == frame.id).expect("an answer on a master id");
    Feedback::decode(&frame.eight().unwrap(), m.limits())
}

/// A master's cycle: a frame to each motor, then `seconds` of physics once
/// all have had theirs, as `--lockstep` runs it.
fn cycle(dock: &mut Drydock, frames: &[(u16, [u8; 8])], seconds: f64) -> Vec<Frame> {
    let answers = frames.iter().filter_map(|&(id, data)| dock.deliver(0, &Frame::new(id, data))).collect();
    assert!(dock.cycle_done(), "every motor had a frame");
    for _ in 0..steps_per_period(dock.timestep(), seconds).unwrap() {
        dock.step();
    }
    answers
}

fn hold(robot: &Robot, p: f32) -> Vec<(u16, [u8; 8])> {
    robot.motors().map(|(_, m)| (m.id, Mit { p, v: 0.0, kp: 60.0, kd: 2.0, t: 0.0 }.encode(m.limits()))).collect()
}

fn enable(robot: &Robot) -> Vec<(u16, [u8; 8])> {
    robot.motors().map(|(_, m)| (m.id, Special::Enable.encode())).collect()
}

#[test]
fn holds_a_position_against_gravity() {
    let (mut dock, robot) = arm();
    cycle(&mut dock, &enable(&robot), 0.001);
    let mut answers = Vec::new();
    for _ in 0..1500 {
        answers = cycle(&mut dock, &hold(&robot, -0.3), 0.001);
    }
    for frame in answers {
        let f = feedback(&robot, frame);
        assert_eq!(f.status, status::ENABLED);
        assert!((f.p + 0.3).abs() < 0.05, "{f:?}");
        assert!(f.v.abs() < 0.02, "{f:?}");
        // Holding up the arm takes torque.
        assert!(f.t.abs() > 0.05, "{f:?}");
    }
}

#[test]
fn a_disabled_arm_falls() {
    let (mut dock, robot) = arm();
    let refresh: Vec<_> = robot.motors().map(|(_, m)| (0x7FF, Register::Refresh { motor: m.id }.encode())).collect();
    let mut answers = Vec::new();
    for _ in 0..500 {
        answers = cycle(&mut dock, &refresh, 0.001);
    }
    let shoulder = feedback(&robot, answers[0]);
    assert_eq!(shoulder.status, status::DISABLED);
    // Zero, to a 12-bit quantum: the packing has no exact zero.
    assert!(shoulder.t.abs() < 56.0 / 4095.0, "{shoulder:?}");
    // Positive about y is down.
    assert!(shoulder.p > 0.5, "{shoulder:?}");
}

#[test]
fn a_motor_left_without_frames_lets_go() {
    let (mut dock, robot) = arm();
    cycle(&mut dock, &enable(&robot), 0.001);
    let hold = hold(&robot, 0.0);
    cycle(&mut dock, &hold, 0.001);
    // The elbow's timeout is 100 ms; the shoulder has none, and keeps
    // holding on its last command.
    for _ in 0..200 {
        dock.step();
    }
    let answers = cycle(&mut dock, &[(SHOULDER, hold[0].1), (ELBOW, Special::Enable.encode())], 0.001);
    assert_eq!(feedback(&robot, answers[0]).status, status::ENABLED);
    assert_eq!(feedback(&robot, answers[1]).status, status::COMMUNICATION_LOST);
    let answers = cycle(&mut dock, &[(SHOULDER, hold[0].1), (ELBOW, Special::ClearError.encode())], 0.001);
    assert_eq!(feedback(&robot, answers[1]).status, status::DISABLED);
}

#[test]
fn registers_answer_on_the_master_id() {
    let (mut dock, _) = arm();
    let reply =
        dock.deliver(0, &Frame::new(0x7FF, Register::Read { motor: ELBOW, rid: reg::MST_ID }.encode())).unwrap();
    assert_eq!(reply.id, 0x12);
    let reply = RegisterReply::decode(&reply.eight().unwrap()).unwrap();
    assert_eq!((reply.motor, reply.value), (ELBOW, 0x12u32.to_le_bytes()));
    // Someone else's motor: no answer.
    assert!(dock.deliver(0, &Frame::new(0x7FF, Register::Read { motor: 0x05, rid: reg::PMAX }.encode())).is_none());
}

#[test]
fn lockstep_runs_are_the_same() {
    let run = || {
        let (mut dock, robot) = arm();
        let mut all = cycle(&mut dock, &enable(&robot), 0.001);
        for i in 0..500 {
            let p = 0.4 * (i as f32 * 0.01).sin();
            all.extend(cycle(&mut dock, &hold(&robot, p), 0.001));
        }
        all
    };
    assert_eq!(run(), run());
}

#[test]
fn a_classic_robot_ignores_can_fd_frames() {
    let (mut dock, _) = arm();
    let enable = Frame::new(SHOULDER, Special::Enable.encode());
    assert!(dock.deliver(0, &enable.fd(true)).is_none());
    let answer = dock.deliver(0, &enable).unwrap();
    assert!(!answer.fd);
}

#[test]
fn a_robot_on_can_fd_answers_in_it_and_understands_classic_frames() {
    let mut robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/arm/robot.yml")).unwrap();
    robot.buses[0].fd = true;
    let mut dock = Drydock::load(&robot).unwrap();
    let enable = Frame::new(SHOULDER, Special::Enable.encode());
    for frame in [enable.fd(true), enable] {
        let answer = dock.deliver(0, &frame).unwrap();
        assert!(answer.fd);
        assert_eq!(feedback(&robot, answer).status, status::ENABLED);
    }
}

#[test]
fn a_dead_motor_lets_go_though_its_firmware_is_enabled() {
    let (mut dock, robot) = arm_with("faults: [{ joint: shoulder, fault: dead, at: 0.5 s }]");
    cycle(&mut dock, &enable(&robot), 0.001);
    let mut answers = Vec::new();
    for _ in 0..1500 {
        answers = cycle(&mut dock, &hold(&robot, -0.3), 0.001);
    }
    let shoulder = feedback(&robot, answers[0]);
    assert_eq!(shoulder.status, status::ENABLED);
    assert!(shoulder.t.abs() < 0.02, "it gives nothing: {shoulder:?}");
    assert!(shoulder.p > 0.5, "the arm hangs: {shoulder:?}");
}

#[test]
fn a_fault_that_lasts_ends_and_says_when() {
    let (mut dock, robot) = arm_with("faults: [{ joint: shoulder, fault: dead, at: 0.5 s, lasts: 300 ms }]");
    cycle(&mut dock, &enable(&robot), 0.001);
    let mut answers = Vec::new();
    for _ in 0..1500 {
        answers = cycle(&mut dock, &hold(&robot, -0.3), 0.001);
    }
    let shoulder = feedback(&robot, answers[0]);
    assert!((shoulder.p + 0.3).abs() < 0.05, "holding again: {shoulder:?}");
    let happened = dock.happenings();
    assert_eq!(happened.len(), 2, "{happened:?}");
    assert!(matches!(happened[0].what, Change::Breaks { .. }) && (happened[0].at - 0.5).abs() < 1e-3, "{happened:?}");
    assert!(happened[1].what == Change::Recovers && (happened[1].at - 0.8).abs() < 1e-3, "{happened:?}");
    assert_eq!(dock.joint_name(happened[0].motor), "shoulder");
}

#[test]
fn a_frozen_encoder_reads_the_same_wherever_the_joint_goes() {
    let (mut dock, robot) = arm_with("faults: [{ joint: shoulder, fault: freeze, at: 1 s }]");
    cycle(&mut dock, &enable(&robot), 0.001);
    let mut answers = Vec::new();
    for _ in 0..1500 {
        answers = cycle(&mut dock, &hold(&robot, -0.3), 0.001);
    }
    let frozen = feedback(&robot, answers[0]);
    for _ in 0..500 {
        answers = cycle(&mut dock, &hold(&robot, 0.3), 0.001);
    }
    let later = feedback(&robot, answers[0]);
    assert_eq!(later.p, frozen.p, "{frozen:?} {later:?}");
    // Still, to a 12-bit quantum: the packing has no exact zero.
    assert!(later.v.abs() < 16.0 / 4095.0, "{later:?}");
    // Its law, closed on the frozen reading, pushes as hard as it may.
    assert!(later.t > 20.0, "{later:?}");
}

#[test]
fn noisy_encoders_repeat_with_their_seed() {
    let run = |noise: f64| {
        let mut robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/arm/robot.yml")).unwrap();
        for m in &mut robot.buses[0].motors {
            m.encoder.noise = noise;
            m.encoder.quantum = 0.0004;
        }
        let mut dock = Drydock::load(&robot).unwrap();
        let mut all = cycle(&mut dock, &enable(&robot), 0.001);
        for _ in 0..300 {
            all.extend(cycle(&mut dock, &hold(&robot, 0.2), 0.001));
        }
        all
    };
    assert_eq!(run(0.002), run(0.002));
    assert_ne!(run(0.002), run(0.0));
}
