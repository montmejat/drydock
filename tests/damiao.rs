//! Damiao's protocol, feature by feature: what drydock's motors do, each
//! shown by a test over the bus's frames, and what they don't, as ignored
//! tests that say why. CI turns this file's results into the support table,
//! on each run's summary page (`cargo test --test damiao` lists the same): a
//! feature is supported when a test here passes, so the table can't claim
//! more than drydock does.
//!
//! Names start with what they're about: `mit_`, `special_` (enable,
//! disable, zero, clear an error), `register_`, `status_`, `bus_`. Features come from Damiao's manual and their reference
//! driver (`DM_CAN.py`).

use drydock::can::timing::{bits, Crossed, From, Rates, Timeline};
use drydock::can::Frame;
use drydock::damiao::{reg, status, Feedback, Limits, Mit, Register, RegisterReply, Special};
use drydock::dock::Drydock;
use drydock::robot::Robot;
use drydock::scenario::Scenario;

/// A DM4310 on a slide, alone on its bus, without a timeout.
const ID: u16 = 0x08;
const MASTER: u16 = 0x18;

fn slide() -> (Drydock, Limits) {
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/slide/robot.yml")).unwrap();
    let limits = robot.buses[0].motors[0].limits();
    (Drydock::load(&robot).unwrap(), limits)
}

fn send(dock: &mut Drydock, id: u16, data: [u8; 8]) -> Option<Frame> {
    dock.deliver(0, &Frame::new(id, data))
}

fn feedback(frame: Frame, limits: Limits) -> Feedback {
    assert_eq!(frame.id, MASTER, "answered on the master id");
    Feedback::decode(&frame.eight().unwrap(), limits)
}

fn command(p: f32, kp: f32, kd: f32, limits: Limits) -> [u8; 8] {
    Mit { p, v: 0.0, kp, kd, t: 0.0 }.encode(limits)
}

fn read(dock: &mut Drydock, rid: u8) -> [u8; 4] {
    let reply = send(dock, 0x7FF, Register::Read { motor: ID, rid }.encode()).expect("an answer");
    assert_eq!(reply.id, MASTER);
    let reply = RegisterReply::decode(&reply.eight().unwrap()).unwrap();
    assert_eq!((reply.motor, reply.rid), (ID, rid));
    reply.value
}

/* ── MIT mode ── */

#[test]
fn mit_commands_are_answered_with_feedback_on_the_master_id() {
    let (mut dock, limits) = slide();
    let answer = send(&mut dock, ID, command(0.0, 0.0, 0.0, limits)).expect("an answer to every frame");
    let f = feedback(answer, limits);
    assert_eq!(f.id, (ID & 0xF) as u8);
}

#[test]
fn mit_commands_drive_the_joint_by_kp_and_kd() {
    let (mut dock, limits) = slide();
    send(&mut dock, ID, Special::Enable.encode());
    let mut last = None;
    for _ in 0..4000 {
        last = send(&mut dock, ID, command(-0.5, 200.0, 2.0, limits));
        dock.step();
    }
    let f = feedback(last.unwrap(), limits);
    assert!((f.p + 0.5).abs() < 0.01, "{f:?}");
}

#[test]
fn mit_values_outside_the_ranges_are_clamped() {
    let limits = Limits { p: 12.5, v: 30.0, t: 10.0 };
    let m = Mit::decode(&Mit { p: 100.0, v: -100.0, kp: 900.0, kd: -1.0, t: 50.0 }.encode(limits), limits);
    assert_eq!((m.p, m.v, m.t), (12.5, -30.0, 10.0));
    assert_eq!((m.kp, m.kd), (500.0, 0.0));
}

/// `seconds` of physics, the motor sent `frame` each step; its last answer.
fn hold(dock: &mut Drydock, frame: [u8; 8], seconds: f64, limits: Limits) -> Feedback {
    let mut last = None;
    for _ in 0..(seconds / dock.timestep()).round() as usize {
        last = send(dock, ID, frame);
        dock.step();
    }
    feedback(last.unwrap(), limits)
}

fn torque(t: f32, limits: Limits) -> [u8; 8] {
    Mit { p: 0.0, v: 0.0, kp: 0.0, kd: 0.0, t }.encode(limits)
}

#[test]
fn mit_feedback_reports_temperatures_that_rise_with_torque() {
    let (mut dock, limits) = slide();
    let cold = feedback(send(&mut dock, ID, Special::Enable.encode()).unwrap(), limits);
    assert_eq!((cold.t_mos, cold.t_rotor), (25, 25));
    // Its rated torque, a third of its peak: warmer, and it stays enabled.
    let warm = hold(&mut dock, torque(limits.t / 3.0, limits), 10.0, limits);
    assert_eq!(warm.status, status::ENABLED);
    assert!(warm.t_mos > 40 && warm.t_rotor > 30, "{warm:?}");
}

/* ── Special frames ── */

#[test]
fn special_enable_and_disable() {
    let (mut dock, limits) = slide();
    let f = feedback(send(&mut dock, ID, Special::Enable.encode()).unwrap(), limits);
    assert_eq!(f.status, status::ENABLED);
    let f = feedback(send(&mut dock, ID, Special::Disable.encode()).unwrap(), limits);
    assert_eq!(f.status, status::DISABLED);
}

#[test]
fn special_a_disabled_motor_gives_no_torque() {
    let (mut dock, limits) = slide();
    let mut last = None;
    for _ in 0..200 {
        last = send(&mut dock, ID, command(-0.5, 200.0, 2.0, limits));
        dock.step();
    }
    let f = feedback(last.unwrap(), limits);
    assert_eq!(f.status, status::DISABLED);
    // Zero, to a 12-bit quantum.
    assert!(f.t.abs() <= 2.0 * limits.t / 4095.0, "{f:?}");
}

#[test]
fn special_zero_makes_the_present_position_zero() {
    let (mut dock, limits) = slide();
    let before = feedback(send(&mut dock, ID, Special::Disable.encode()).unwrap(), limits);
    assert!(before.p.abs() > 0.5, "the slide starts away from the motor's zero: {before:?}");
    let after = feedback(send(&mut dock, ID, Special::Zero.encode()).unwrap(), limits);
    assert!(after.p.abs() < 1e-3, "{after:?}");
}

#[test]
fn special_clear_error_after_communication_lost() {
    // The example arm's elbow lets go after 100 ms without a frame.
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/arm/robot.yml")).unwrap();
    let elbow = robot.buses[0].motors.iter().find(|m| m.id == 0x02).unwrap();
    let limits = elbow.limits();
    let mut dock = Drydock::load(&robot).unwrap();
    dock.deliver(0, &Frame::new(0x02, Special::Enable.encode()));
    // 200 ms of silence, at the arm's 0.5 ms steps.
    for _ in 0..400 {
        dock.step();
    }
    let status_of = |dock: &mut Drydock| {
        let f = dock.deliver(0, &Frame::new(0x7FF, Register::Refresh { motor: 0x02 }.encode())).unwrap();
        Feedback::decode(&f.eight().unwrap(), limits).status
    };
    assert_eq!(status_of(&mut dock), status::COMMUNICATION_LOST);
    dock.deliver(0, &Frame::new(0x02, Special::ClearError.encode()));
    assert_eq!(status_of(&mut dock), status::DISABLED);
}

/* ── Registers, through 0x7FF ── */

#[test]
fn register_reads_and_writes_of_the_over_temperature_threshold() {
    let (mut dock, _) = slide();
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::OT_VALUE)), 120.0);
    send(&mut dock, 0x7FF, Register::Write { motor: ID, rid: reg::OT_VALUE, value: 150.0f32.to_le_bytes() }.encode());
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::OT_VALUE)), 150.0);
    // Out of 80 to 200 °C: refused, and the answer says what it kept.
    send(&mut dock, 0x7FF, Register::Write { motor: ID, rid: reg::OT_VALUE, value: 50.0f32.to_le_bytes() }.encode());
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::OT_VALUE)), 150.0);
}

#[test]
fn register_reads_of_the_ranges_ids_and_mode() {
    let (mut dock, limits) = slide();
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::PMAX)), limits.p);
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::VMAX)), limits.v);
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::TMAX)), limits.t);
    assert_eq!(u32::from_le_bytes(read(&mut dock, reg::MST_ID)), MASTER as u32);
    assert_eq!(u32::from_le_bytes(read(&mut dock, reg::ESC_ID)), ID as u32);
    assert_eq!(u32::from_le_bytes(read(&mut dock, reg::CTRL_MODE)), reg::MIT_MODE);
}

#[test]
fn register_writes_of_the_ranges_change_the_packing() {
    let (mut dock, _) = slide();
    send(&mut dock, 0x7FF, Register::Write { motor: ID, rid: reg::VMAX, value: 10.0f32.to_le_bytes() }.encode());
    assert_eq!(f32::from_le_bytes(read(&mut dock, reg::VMAX)), 10.0);
}

#[test]
fn register_refresh_asks_for_feedback() {
    let (mut dock, limits) = slide();
    let f = feedback(send(&mut dock, 0x7FF, Register::Refresh { motor: ID }.encode()).unwrap(), limits);
    assert_eq!(f.status, status::DISABLED);
}

#[test]
fn register_save_is_accepted() {
    let (mut dock, _) = slide();
    send(&mut dock, 0x7FF, Register::Save { motor: ID }.encode());
    assert_eq!(u32::from_le_bytes(read(&mut dock, reg::ESC_ID)), ID as u32);
}

#[test]
fn register_requests_for_another_motor_go_unanswered() {
    let (mut dock, _) = slide();
    assert!(send(&mut dock, 0x7FF, Register::Read { motor: 0x05, rid: reg::PMAX }.encode()).is_none());
}

/* ── The bus ── */

#[test]
fn bus_lower_ids_win_arbitration() {
    let mut bus = Timeline::new(Rates { nominal: 1e6, data: 5e6 });
    bus.push(Frame::new(0x12, [0; 8]), From::Motor, 0.0);
    bus.push(Frame::new(0x02, [0; 8]), From::Master, 0.0);
    let crossed: Vec<Crossed> = bus.advance(1.0);
    assert_eq!(crossed.iter().map(|c| c.frame.id).collect::<Vec<_>>(), [0x02, 0x12]);
}

#[test]
fn bus_can_fd_frames_cross_faster_than_classic_ones() {
    let rates = Rates { nominal: 1e6, data: 5e6 };
    let classic = bits(&Frame::new(0x01, [0x55; 8])).seconds(rates);
    let fd = bits(&Frame::new(0x01, [0x55; 8]).fd(true)).seconds(rates);
    assert!(fd < classic, "FD {fd} s, classic {classic} s");
}

/// The slide, with what a scenario's text says happens to its motor.
fn slide_with(scenario: &str) -> (Drydock, Limits) {
    let (mut dock, limits) = slide();
    dock.set_scenario(Scenario::parse(scenario, &["slide".to_string()]).unwrap());
    (dock, limits)
}

#[test]
fn bus_a_disconnected_motor_neither_hears_nor_answers_until_reconnected() {
    let (mut dock, _) = slide_with("faults: [{ joint: slide, fault: disconnected, at: 0 s, lasts: 100 ms }]");
    dock.step();
    assert!(send(&mut dock, ID, Special::Enable.encode()).is_none());
    assert!(send(&mut dock, 0x7FF, Register::Refresh { motor: ID }.encode()).is_none());
    for _ in 0..(0.2 / dock.timestep()) as usize {
        dock.step();
    }
    let f = send(&mut dock, 0x7FF, Register::Refresh { motor: ID }.encode()).expect("reconnected");
    assert_eq!(f.id, MASTER);
    // The enable frame it missed: it's still disabled.
    let f = Feedback::decode(&f.eight().unwrap(), slide().1);
    assert_eq!(f.status, status::DISABLED);
}

#[test]
fn bus_a_lossy_link_loses_about_a_tenth_of_frames_each_way() {
    let (mut dock, limits) = slide_with("faults: [{ joint: slide, fault: lossy, at: 0 s }]");
    dock.step();
    let answered = (0..2000).filter(|_| send(&mut dock, ID, command(0.0, 0.0, 0.0, limits)).is_some()).count();
    // 0.9 × 0.9 of them, give or take: lost on the way there, or back.
    assert!((1520..1720).contains(&answered), "{answered} of 2000");
}

/* ── Not simulated yet ── */

#[test]
#[ignore = "not simulated: position-velocity frames (id + 0x100) are ignored, with a note"]
fn mit_position_velocity_mode() {}

#[test]
#[ignore = "not simulated: velocity frames (id + 0x200) are ignored, with a note"]
fn mit_speed_mode() {}

#[test]
#[ignore = "not simulated: position-force frames (id + 0x300) are ignored, with a note"]
fn mit_force_position_mode() {}

#[test]
#[ignore = "not simulated: writing another mode is refused; the answer says MIT mode (1)"]
fn register_writes_of_the_control_mode() {}

#[test]
#[ignore = "not simulated: ids can't be rewritten over the bus; they're the robot file's"]
fn register_writes_of_the_ids() {}

#[test]
#[ignore = "not simulated: TIMEOUT (9) gets no answer, its units aren't in Damiao's sources; the robot file's timeout_ms sets it"]
fn register_writes_of_the_timeout() {}

#[test]
#[ignore = "not simulated: registers other than OT_Value, MST_ID, ESC_ID, CTRL_MODE, PMAX, VMAX and TMAX get no answer, with a note"]
fn register_reads_of_the_other_registers() {}

#[test]
fn status_overheating_stops_the_motor_until_it_cools_and_is_cleared() {
    let (mut dock, limits) = slide();
    send(&mut dock, ID, Special::Enable.encode());
    // At its peak torque, three times its rating, the drive overheats first.
    let hot = hold(&mut dock, torque(limits.t, limits), 8.0, limits);
    assert_eq!(hot.status, status::MOS_OVERTEMPERATURE, "{hot:?}");
    // It tripped at 120 °C a few seconds in, and has been cooling since.
    assert!(hot.t_mos > 100, "{hot:?}");
    let f = hold(&mut dock, torque(limits.t, limits), 0.1, limits);
    assert!(f.t.abs() <= 2.0 * limits.t / 4095.0, "no torque while in error: {f:?}");
    // Cleared, once below its threshold, it runs again; pushed as hard, it
    // trips again, sooner, as it hasn't cooled all the way.
    send(&mut dock, ID, Special::ClearError.encode());
    send(&mut dock, ID, Special::Enable.encode());
    let again = hold(&mut dock, torque(limits.t, limits), 0.5, limits);
    assert_eq!(again.status, status::ENABLED, "{again:?}");
    let again = hold(&mut dock, torque(limits.t, limits), 3.0, limits);
    assert_eq!(again.status, status::MOS_OVERTEMPERATURE, "{again:?}");
}

#[test]
#[ignore = "not simulated: no power supply, so overvoltage, undervoltage, overcurrent and overload are never reported"]
fn status_electrical_faults() {}

#[test]
#[ignore = "not simulated: the bus's bit rates are the robot file's"]
fn bus_bit_rate_register() {}

#[test]
#[ignore = "not simulated: a lost frame is just lost; no error frames, retransmissions, error counters or bus-off"]
fn bus_error_frames_and_bus_off() {}
