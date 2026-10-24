//! The example arm in MuJoCo, frame by frame, without a bus.

use drydock::can::Frame;
use drydock::damiao::{reg, status, Feedback, Mit, Register, RegisterReply, Special};
use drydock::dock::Drydock;
use drydock::robot::Robot;
use drydock::run::steps_per_period;

const SHOULDER: u16 = 0x01;
const ELBOW: u16 = 0x02;

fn arm() -> (Drydock, Robot) {
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/arm/robot.yml")).unwrap();
    (Drydock::load(&robot).unwrap(), robot)
}

fn feedback(robot: &Robot, frame: Frame) -> Feedback {
    let m = robot.motors.iter().find(|m| m.master_id == frame.id).expect("an answer on a master id");
    Feedback::decode(&frame.data, m.model.limits())
}

/// A master's cycle: a frame to each motor, then `seconds` of physics once
/// all have had theirs, as `--lockstep` runs it.
fn cycle(dock: &mut Drydock, frames: &[(u16, [u8; 8])], seconds: f64) -> Vec<Frame> {
    let answers = frames.iter().filter_map(|&(id, data)| dock.handle(&Frame::new(id, data))).collect();
    assert!(dock.cycle_done(), "every motor had a frame");
    for _ in 0..steps_per_period(dock.timestep(), seconds).unwrap() {
        dock.step();
    }
    answers
}

fn hold(robot: &Robot, p: f32) -> Vec<(u16, [u8; 8])> {
    robot.motors.iter().map(|m| (m.id, Mit { p, v: 0.0, kp: 60.0, kd: 2.0, t: 0.0 }.encode(m.model.limits()))).collect()
}

fn enable(robot: &Robot) -> Vec<(u16, [u8; 8])> {
    robot.motors.iter().map(|m| (m.id, Special::Enable.encode())).collect()
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
    let refresh: Vec<_> = robot.motors.iter().map(|m| (0x7FF, Register::Refresh { motor: m.id }.encode())).collect();
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
    let reply = dock.handle(&Frame::new(0x7FF, Register::Read { motor: ELBOW, rid: reg::MST_ID }.encode())).unwrap();
    assert_eq!(reply.id, 0x12);
    let reply = RegisterReply::decode(&reply.data).unwrap();
    assert_eq!((reply.motor, reply.value), (ELBOW, 0x12u32.to_le_bytes()));
    // Someone else's motor: no answer.
    assert!(dock.handle(&Frame::new(0x7FF, Register::Read { motor: 0x05, rid: reg::PMAX }.encode())).is_none());
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
