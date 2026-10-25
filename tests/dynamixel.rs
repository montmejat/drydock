//! Dynamixel's Protocol 2.0 and the XL330, feature by feature: what
//! drydock's servos do, each shown by a test over the bus's packets, and
//! what they don't, as ignored tests that say why. CI turns this file's
//! results into a support table, as it does `tests/damiao.rs`'s.
//!
//! Names start with what they're about: `ping_`, `read_`, `write_`, `sync_`,
//! `bulk_`, `fast_`, `status_` (answers and errors), `servo_` (what it does
//! with the joint), `bus_`. Features come from ROBOTIS's Protocol 2.0 and
//! XL330 e-Manuals, and the servo's behaviour from BAM's model of it.

use drydock::dock::Drydock;
use drydock::dynamixel::bus::Reply;
use drydock::dynamixel::xl330::addr;
use drydock::dynamixel::{error, Chunk, Decoder, FastReply, Instruction, Packet, Region, Status, BROADCAST_ID};
use drydock::robot::Robot;
use drydock::scenario::Scenario;

/// Two XL330s, ids 1 and 2, arms hanging down, on 5 V.
fn pair() -> Drydock {
    let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/xl330/robot.yml")).unwrap();
    let mut dock = Drydock::load(&robot).unwrap();
    dock.step();
    dock
}

fn packet(id: u8, instruction: &Instruction) -> Packet {
    let mut d = Decoder::default();
    d.push(&instruction.encode(id));
    d.next_packet().unwrap().unwrap()
}

fn send(dock: &mut Drydock, id: u8, instruction: Instruction) -> Vec<Reply> {
    dock.deliver_packet(0, &packet(id, &instruction))
}

fn status(reply: &Reply) -> Status {
    let mut d = Decoder::default();
    d.push(&reply.bytes);
    Status::decode(&d.next_packet().unwrap().unwrap()).unwrap()
}

/// The one status packet answering an instruction to one servo.
fn one(dock: &mut Drydock, id: u8, instruction: Instruction) -> Status {
    let replies = send(dock, id, instruction);
    assert_eq!(replies.len(), 1, "one answer: {replies:?}");
    status(&replies[0])
}

fn read(dock: &mut Drydock, id: u8, address: u16, length: u16) -> Vec<u8> {
    let s = one(dock, id, Instruction::Read { address, length });
    assert_eq!(s.error, 0, "{s:?}");
    s.params
}

fn write(dock: &mut Drydock, id: u8, address: u16, data: &[u8]) -> u8 {
    one(dock, id, Instruction::Write { address, data: data.to_vec() }).error
}

fn i32_at(b: &[u8]) -> i32 {
    i32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

fn steps(dock: &mut Drydock, seconds: f64) {
    for _ in 0..(seconds / dock.timestep()).round() as usize {
        dock.step();
    }
}

/* ── Ping ── */

#[test]
fn ping_answers_with_the_model_number_and_firmware_version() {
    let mut dock = pair();
    let s = one(&mut dock, 1, Instruction::Ping);
    assert_eq!((s.id, s.error, s.params.as_slice()), (1, 0, &[0xB0, 0x04, 46][..]), "1200, v46");
    assert!(send(&mut dock, 3, Instruction::Ping).is_empty(), "no servo 3");
}

#[test]
fn ping_broadcast_is_answered_by_each_servo_in_turn() {
    let mut dock = pair();
    let replies = send(&mut dock, BROADCAST_ID, Instruction::Ping);
    assert_eq!(replies.iter().map(|r| status(r).id).collect::<Vec<_>>(), [1, 2]);
    // Each after its 500 µs return delay, the second after the first's 14 bytes too.
    assert!((replies[0].after - 500e-6).abs() < 1e-9);
    assert!((replies[1].after - (1000e-6 + 14.0 * 10.0 / 1e6)).abs() < 1e-9, "{}", replies[1].after);
}

/* ── Reads and writes ── */

#[test]
fn read_any_part_of_the_control_table() {
    let mut dock = pair();
    assert_eq!(read(&mut dock, 2, addr::ID, 1), [2]);
    assert_eq!(read(&mut dock, 1, addr::POSITION_P_GAIN, 2), 400u16.to_le_bytes());
    assert_eq!(read(&mut dock, 1, 0, 7), [0xB0, 0x04, 0, 0, 0, 0, 46]);
    let past = one(&mut dock, 1, Instruction::Read { address: 250, length: 4 });
    assert_eq!(past.error, error::ACCESS);
}

#[test]
fn write_items_whole_within_their_ranges() {
    let mut dock = pair();
    assert_eq!(write(&mut dock, 1, addr::RETURN_DELAY_TIME, &[0]), 0);
    assert_eq!(read(&mut dock, 1, addr::RETURN_DELAY_TIME, 1), [0]);
    assert_eq!(write(&mut dock, 1, addr::MODEL_NUMBER, &[0, 0]), error::ACCESS, "read-only");
    assert_eq!(write(&mut dock, 1, addr::BAUD_RATE, &[9]), error::DATA_RANGE);
    assert_eq!(write(&mut dock, 1, addr::POSITION_P_GAIN, &[1]), error::DATA_LENGTH, "half an item");
    assert_eq!(write(&mut dock, 1, addr::GOAL_POSITION, &5000i32.to_le_bytes()), error::DATA_RANGE);
}

#[test]
fn write_eeprom_only_with_torque_off() {
    let mut dock = pair();
    assert_eq!(write(&mut dock, 1, addr::TORQUE_ENABLE, &[1]), 0);
    assert_eq!(write(&mut dock, 1, addr::RETURN_DELAY_TIME, &[0]), error::ACCESS);
    assert_eq!(write(&mut dock, 1, addr::TORQUE_ENABLE, &[0]), 0);
    assert_eq!(write(&mut dock, 1, addr::RETURN_DELAY_TIME, &[0]), 0);
}

#[test]
fn write_goals_within_the_position_limits() {
    let mut dock = pair();
    assert_eq!(write(&mut dock, 1, addr::MAX_POSITION_LIMIT, &3000i32.to_le_bytes()), 0);
    assert_eq!(write(&mut dock, 1, addr::GOAL_POSITION, &3500i32.to_le_bytes()), error::DATA_LIMIT);
}

#[test]
fn write_registered_then_done_on_action() {
    let mut dock = pair();
    let s = one(&mut dock, 1, Instruction::RegWrite { address: addr::RETURN_DELAY_TIME, data: vec![7] });
    assert_eq!(s.error, 0);
    assert_eq!(read(&mut dock, 1, addr::RETURN_DELAY_TIME, 1), [250], "not yet");
    assert!(send(&mut dock, BROADCAST_ID, Instruction::Action).is_empty());
    assert_eq!(read(&mut dock, 1, addr::RETURN_DELAY_TIME, 1), [7]);
}

/* ── Sync, bulk and fast ── */

#[test]
fn sync_write_writes_each_servo_its_own_value_unanswered() {
    let mut dock = pair();
    let writes = vec![(1, vec![10]), (2, vec![20])];
    assert!(send(
        &mut dock,
        BROADCAST_ID,
        Instruction::SyncWrite { address: addr::RETURN_DELAY_TIME, length: 1, writes }
    )
    .is_empty());
    assert_eq!(
        (read(&mut dock, 1, addr::RETURN_DELAY_TIME, 1), read(&mut dock, 2, addr::RETURN_DELAY_TIME, 1)),
        (vec![10], vec![20])
    );
}

#[test]
fn sync_read_is_answered_by_each_servo_in_turn() {
    let mut dock = pair();
    let ids = vec![2, 1];
    let replies = send(&mut dock, BROADCAST_ID, Instruction::SyncRead { address: addr::ID, length: 1, ids });
    let got: Vec<_> = replies.iter().map(|r| (status(r).id, status(r).params)).collect();
    assert_eq!(got, [(2, vec![2]), (1, vec![1])], "in the order asked");
    assert!(replies[1].after > replies[0].after);
}

#[test]
fn bulk_read_and_bulk_write() {
    let mut dock = pair();
    let chunks = vec![
        Chunk { id: 1, address: addr::RETURN_DELAY_TIME, data: vec![3] },
        Chunk { id: 2, address: addr::POSITION_P_GAIN, data: 200u16.to_le_bytes().to_vec() },
    ];
    assert!(send(&mut dock, BROADCAST_ID, Instruction::BulkWrite(chunks)).is_empty());
    let regions = vec![
        Region { id: 1, address: addr::RETURN_DELAY_TIME, length: 1 },
        Region { id: 2, address: addr::POSITION_P_GAIN, length: 2 },
    ];
    let replies = send(&mut dock, BROADCAST_ID, Instruction::BulkRead(regions));
    assert_eq!(replies.iter().map(|r| status(r).params).collect::<Vec<_>>(), [vec![3], vec![200, 0]]);
}

#[test]
fn fast_sync_read_answers_in_one_packet() {
    let mut dock = pair();
    let ids = vec![1, 2];
    let replies =
        send(&mut dock, BROADCAST_ID, Instruction::FastSyncRead { address: addr::PRESENT_POSITION, length: 4, ids });
    assert_eq!(replies.len(), 1);
    let fast = FastReply::decode(&replies[0].bytes, &[4, 4]).expect("one packet, every CRC right");
    let hanging = (2048.0 + std::f64::consts::FRAC_PI_2 / (std::f64::consts::TAU / 4096.0)).round() as i32;
    for a in &fast.answers {
        assert_eq!((a.error, i32_at(&a.data)), (0, hanging), "{a:?}");
    }
}

#[test]
fn fast_bulk_read_answers_in_one_packet() {
    let mut dock = pair();
    let regions = vec![
        Region { id: 1, address: addr::ID, length: 1 },
        Region { id: 2, address: addr::PRESENT_INPUT_VOLTAGE, length: 2 },
    ];
    let replies = send(&mut dock, BROADCAST_ID, Instruction::FastBulkRead(regions));
    let fast = FastReply::decode(&replies[0].bytes, &[1, 2]).unwrap();
    assert_eq!((fast.answers[0].data.clone(), fast.answers[1].data.clone()), (vec![1], 50u16.to_le_bytes().to_vec()));
}

#[test]
fn fast_sync_read_of_a_missing_servo_breaks_the_answer() {
    let mut dock = pair();
    let ids = vec![1, 7];
    assert!(send(&mut dock, BROADCAST_ID, Instruction::FastSyncRead { address: addr::ID, length: 1, ids }).is_empty());
}

/* ── Answers and errors ── */

#[test]
fn status_return_level_says_what_is_answered() {
    let mut dock = pair();
    // Level 1: pings and reads. Whether the write that sets it is answered isn't in the spec.
    send(&mut dock, 1, Instruction::Write { address: 68, data: vec![1] });
    assert!(send(&mut dock, 1, Instruction::Write { address: addr::RETURN_DELAY_TIME, data: vec![0] }).is_empty());
    assert_eq!(read(&mut dock, 1, addr::RETURN_DELAY_TIME, 1), [0], "written, unanswered");
    assert_eq!(send(&mut dock, 1, Instruction::Ping).len(), 1);
}

#[test]
fn status_unknown_instructions_get_the_instruction_error() {
    let mut dock = pair();
    let p = Packet { id: 1, instruction: 0x20, params: vec![] };
    let replies = dock.deliver_packet(0, &p);
    assert_eq!(status(&replies[0]).error, error::INSTRUCTION);
}

#[test]
fn status_reboot_brings_the_ram_back_and_keeps_the_eeprom() {
    let mut dock = pair();
    write(&mut dock, 1, addr::RETURN_DELAY_TIME, &[0]);
    write(&mut dock, 1, addr::TORQUE_ENABLE, &[1]);
    assert_eq!(one(&mut dock, 1, Instruction::Reboot).error, 0);
    assert_eq!(
        (read(&mut dock, 1, addr::TORQUE_ENABLE, 1), read(&mut dock, 1, addr::RETURN_DELAY_TIME, 1)),
        (vec![0], vec![0])
    );
}

#[test]
fn status_factory_reset_keeps_what_its_option_says() {
    let mut dock = pair();
    write(&mut dock, 2, addr::RETURN_DELAY_TIME, &[0]);
    one(&mut dock, 2, Instruction::FactoryReset { option: 0x02 });
    assert_eq!((read(&mut dock, 2, addr::ID, 1), read(&mut dock, 2, addr::RETURN_DELAY_TIME, 1)), (vec![2], vec![250]));
}

/* ── The servo on its joint ── */

#[test]
fn servo_position_control_lifts_and_holds_against_gravity() {
    let mut dock = pair();
    write(&mut dock, 1, addr::TORQUE_ENABLE, &[1]);
    write(&mut dock, 1, addr::GOAL_POSITION, &2048i32.to_le_bytes());
    steps(&mut dock, 1.0);
    let at = i32_at(&read(&mut dock, 1, addr::PRESENT_POSITION, 4));
    // A P loop holding 100 g at 10 cm sags short of its goal: about 0.1 rad, 70 pulses.
    assert!((40..120).contains(&(at - 2048)), "{at}");
    let current = i16::from_le_bytes(read(&mut dock, 1, addr::PRESENT_CURRENT, 2).try_into().unwrap());
    assert!((100..400).contains(&current.abs()), "{current} mA");
    // The other servo, torque off, still hangs.
    assert!(i32_at(&read(&mut dock, 2, addr::PRESENT_POSITION, 4)) > 3000);
}

#[test]
fn servo_current_limit_caps_its_pull() {
    let mut dock = pair();
    write(&mut dock, 1, addr::CURRENT_LIMIT, &300u16.to_le_bytes());
    write(&mut dock, 1, addr::TORQUE_ENABLE, &[1]);
    write(&mut dock, 1, addr::GOAL_POSITION, &2048i32.to_le_bytes());
    steps(&mut dock, 0.05);
    let current = i16::from_le_bytes(read(&mut dock, 1, addr::PRESENT_CURRENT, 2).try_into().unwrap());
    assert!(current.abs() <= 300, "{current} mA");
}

#[test]
fn servo_above_its_voltage_limit_shuts_down_unless_shutdown_says_not_to() {
    let mut dock = pair();
    dock.set_scenario(Scenario::parse("faults: [{ supply: 7.4 V, at: 0 s }]", &["a".into(), "b".into()]).unwrap());
    write(&mut dock, 2, addr::SHUTDOWN, &[52]);
    write(&mut dock, 1, addr::TORQUE_ENABLE, &[1]);
    write(&mut dock, 2, addr::TORQUE_ENABLE, &[1]);
    steps(&mut dock, 0.01);
    let s = one(&mut dock, 1, Instruction::Read { address: addr::HARDWARE_ERROR_STATUS, length: 1 });
    assert_eq!((s.error, s.params.as_slice()), (error::ALERT, &[1u8][..]), "alert, input voltage");
    let torque =
        |dock: &mut Drydock, id| one(dock, id, Instruction::Read { address: addr::TORQUE_ENABLE, length: 1 }).params;
    assert_eq!(torque(&mut dock, 1), [0]);
    assert_eq!(torque(&mut dock, 2), [1], "the Microduck's Shutdown, 52");
    one(&mut dock, 1, Instruction::Reboot);
    assert_eq!(one(&mut dock, 1, Instruction::Ping).error, 0, "a reboot clears it");
}

/* ── The bus ── */

#[test]
fn bus_a_servo_at_another_baud_rate_neither_hears_nor_answers() {
    let mut dock = pair();
    assert_eq!(write(&mut dock, 1, addr::BAUD_RATE, &[1]), 0, "answered at the old rate");
    assert!(send(&mut dock, 1, Instruction::Ping).is_empty());
}

#[test]
fn bus_a_disconnected_servo_neither_hears_nor_answers() {
    let mut dock = pair();
    dock.set_scenario(
        Scenario::parse("faults: [{ joint: a, fault: disconnected, at: 0 s }]", &["a".into(), "b".into()]).unwrap(),
    );
    dock.step();
    assert!(send(&mut dock, 1, Instruction::Ping).is_empty());
    assert_eq!(send(&mut dock, 2, Instruction::Ping).len(), 1);
}

/* ── Not simulated yet ── */

#[test]
#[ignore = "not simulated: position control (mode 3) only; velocity, current, current-based position, extended position and PWM modes keep the mode written but don't change how the servo drives"]
fn servo_other_operating_modes() {}

#[test]
#[ignore = "not simulated: the position loop is BAM's, P only; the I and D gains, feedforward, profiles and the velocity loop are kept as written but change nothing"]
fn servo_position_i_and_d_gains_feedforward_and_profiles() {}

#[test]
#[ignore = "not simulated: no heat, so no overheating error, and the temperature reads 25 °C; nor overload or electrical shock"]
fn status_overheating_overload_and_electrical_shock() {}

#[test]
#[ignore = "not simulated: the bus watchdog is kept as written but never fires"]
fn servo_bus_watchdog() {}

#[test]
#[ignore = "not simulated: indirect addresses are kept as written, but indirect data doesn't follow them"]
fn read_indirect_addresses() {}

#[test]
#[ignore = "not simulated: the secondary (shadow) id, Control Table Backup (0x20), and the XL330-M077"]
fn bus_secondary_ids_backup_and_the_m077() {}
