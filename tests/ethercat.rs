//! EtherCAT, feature by feature: what drydock's CiA 402 drives on their
//! ET1100s do, each shown by a test with a real master (ethercrab) on the
//! other end of a virtual cable, and what they don't, as ignored tests that
//! say why.
//!
//! Each test runs in a network namespace of its own (`unshare -rn`): it
//! makes a veth pair there, `ecm0` for the master and `ecs0` for drydock,
//! so it needs neither root nor the host's interfaces.
//!
//! Names start with what they're about: `bus_` (the line, the chip),
//! `al_` (the EtherCAT states), `coe_` (the mailbox), `cia402_` (the drive).

use std::process::Command;
use std::time::Duration;

use drydock::dock::Drydock;
use drydock::ethercat::Socket;
use drydock::robot::Robot;
use drydock::run::{self, EthercatLine};
use ethercrab::error::{CoeAbortCode, Error, MailboxError};
use ethercrab::std::{ethercat_now, tx_rx_task};
use ethercrab::subdevice_group::{Op, PreOp, SubDeviceGroup};
use ethercrab::{AlStatusCode, MainDevice, MainDeviceConfig, PduStorage, SubDeviceState, Timeouts};

const DRIVES: usize = 3;
/// Room for the drives, as ethercrab counts it: a power of two.
const MAX: usize = 4;
const PDI: usize = 128;
const OUT: usize = 13;

type Group<S> = SubDeviceGroup<MAX, PDI, ethercrab::DefaultLock, S>;

static PDU: PduStorage<16, { PduStorage::element_size(1100) }> = PduStorage::new();

/// Runs `test` in a network namespace of its own, with the veth pair up and
/// drydock serving the example arm on `ecs0` in real time. The test binary
/// runs itself again, inside, on this one test.
fn sandboxed(name: &str, test: impl FnOnce(&MainDevice)) {
    if std::env::var_os("DRYDOCK_NETNS").is_none() {
        let exe = std::env::current_exe().unwrap();
        let status = Command::new("unshare")
            .args(["-rn", "--"])
            .arg(exe)
            .args([name, "--exact", "--nocapture", "--test-threads=1", "--include-ignored"])
            .env("DRYDOCK_NETNS", "1")
            // Its report on stderr: stdout is the outer run's, one line a
            // test, which CI turns into the support table.
            .stdout(std::io::stderr())
            .status()
            .expect("unshare, from util-linux, runs the test in a network namespace of its own");
        assert!(status.success(), "{name}, in its namespace, failed");
        return;
    }
    for args in [
        &["link", "add", "ecm0", "type", "veth", "peer", "name", "ecs0"][..],
        &["link", "set", "ecm0", "up"],
        &["link", "set", "ecs0", "up"],
    ] {
        assert!(Command::new("ip").args(args).status().unwrap().success(), "ip {}", args.join(" "));
    }
    // The master starts once drydock is serving: frames sent before, while
    // the model loads, would time out.
    let (ready, serving) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
        let line = EthercatLine { name: "ecs0".into(), socket: Socket::open("ecs0").unwrap() };
        let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/ethercat-arm/robot.yml")).unwrap();
        let dock = Drydock::load(&robot).unwrap();
        ready.send(()).unwrap();
        run::realtime(dock, &[], &mut [], &[line], None)
    });
    serving.recv().expect("drydock loads the example arm");

    let (tx, rx, pdu_loop) = PDU.try_split().expect("one master per process");
    std::thread::spawn(move || smol::block_on(tx_rx_task("ecm0", tx, rx).unwrap()));
    let timeouts = Timeouts { mailbox_response: Duration::from_millis(500), ..Default::default() };
    let master = MainDevice::new(pdu_loop, timeouts, MainDeviceConfig::default());
    test(&master);
}

fn init(master: &MainDevice) -> Group<PreOp> {
    smol::block_on(master.init_single_group::<MAX, PDI>(ethercat_now)).expect("the line comes up")
}

/// The statusword's state bits, and the actual position.
fn status(inputs: &[u8]) -> (u16, i32) {
    (u16::from_le_bytes([inputs[0], inputs[1]]) & 0x6F, i32::from_le_bytes(inputs[2..6].try_into().unwrap()))
}

/// Runs cycles of 1 ms: `each(drive, inputs, outputs)` sets the outputs.
fn cycles(master: &MainDevice, group: &Group<Op>, n: usize, mut each: impl FnMut(usize, &[u8], &mut [u8])) {
    smol::block_on(async {
        let mut tick = smol::Timer::interval(Duration::from_millis(1));
        for _ in 0..n {
            group.tx_rx(master).await.expect("a cycle");
            for (i, d) in group.iter(master).enumerate() {
                let mut io = d.io_raw_mut();
                let inputs = io.inputs().to_vec();
                each(i, &inputs, io.outputs());
            }
            smol::stream::StreamExt::next(&mut tick).await;
        }
    });
}

/// Brings every drive to Operation Enabled in CSP, holding where it is, as
/// a master does it: Shutdown, Switch On, then the target set to the
/// actual position and Enable Operation.
fn enable(master: &MainDevice, group: &Group<Op>) {
    let mut seen = [0u16; DRIVES];
    cycles(master, group, 200, |i, inputs, out| {
        let (state, actual) = status(inputs);
        if seen[i] != state {
            eprintln!("drive {i}: statusword state bits {state:#06x}");
            seen[i] = state;
        }
        let cw: u16 = match state {
            0x40 => 0x06,
            0x21 => 0x07,
            _ => 0x0F,
        };
        out[0..2].copy_from_slice(&cw.to_le_bytes());
        if state != 0x27 {
            out[2..6].copy_from_slice(&actual.to_le_bytes());
        }
        out[12] = 8;
    });
}

/// The abort code an SDO transfer ended with, if it was aborted.
fn aborted<T: std::fmt::Debug>(result: Result<T, Error>) -> CoeAbortCode {
    match result {
        Err(Error::Mailbox(MailboxError::Aborted { code, .. })) => code,
        other => panic!("expected an SDO abort, got {other:?}"),
    }
}

/// The actual velocity (0x606C) and torque (0x6077) in the default TxPDO.
fn velocity_torque(inputs: &[u8]) -> (i32, i16) {
    (i32::from_le_bytes(inputs[6..10].try_into().unwrap()), i16::from_le_bytes([inputs[10], inputs[11]]))
}

/// Counts a turn, from the robot file's default.
const TURN: f64 = (1 << 17) as f64;

/* ── the line ── */

#[test]
fn bus_the_master_finds_three_drives_in_the_cables_order() {
    sandboxed("bus_the_master_finds_three_drives_in_the_cables_order", |master| {
        let group = init(master);
        assert_eq!(group.len(), DRIVES);
        for (i, d) in group.iter(master).enumerate() {
            assert_eq!(d.name(), "drydock CiA 402 drive");
            assert_eq!(d.identity().serial, i as u32 + 1, "serial numbers in the robot file's order");
        }
    });
}

/* ── EtherCAT states ── */

#[test]
fn al_the_drives_reach_operational() {
    sandboxed("al_the_drives_reach_operational", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        for d in group.iter(master) {
            let (state, code) = smol::block_on(d.status()).unwrap();
            assert_eq!(state, SubDeviceState::Op, "{code:?}");
            let io = d.io_raw();
            assert_eq!((io.inputs().len(), io.outputs().len()), (15, OUT), "the default PDOs: 15 bytes in, 13 out");
        }
    });
}

#[test]
fn al_pre_op_straight_to_op_is_refused_with_its_code() {
    sandboxed("al_pre_op_straight_to_op_is_refused_with_its_code", |master| {
        let group = init(master);
        let d = group.iter(master).next().unwrap();
        smol::block_on(d.register_write(0x0120u16, 0x08u16)).unwrap();
        // The AL status (0x0130) and its code (0x0134), read raw: ethercrab's
        // `status` turns an error into an `Err`. Pre-Op, with the error bit.
        let al: u16 = smol::block_on(d.register_read(0x0130u16)).unwrap();
        let code: u16 = smol::block_on(d.register_read(0x0134u16)).unwrap();
        assert_eq!((al, code), (0x12, 0x0011));
    });
}

#[test]
fn al_outputs_left_unwritten_trip_the_sm_watchdog_to_safe_op() {
    sandboxed("al_outputs_left_unwritten_trip_the_sm_watchdog_to_safe_op", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        enable(master, &group);
        // The watchdog's default: 100 ms without outputs.
        std::thread::sleep(Duration::from_millis(200));
        for d in group.iter(master) {
            let status = smol::block_on(d.status());
            assert!(matches!(status, Err(Error::SubDevice(AlStatusCode::SyncManagerWatchdog))), "{status:?}");
            let al: u16 = smol::block_on(d.register_read(0x0130u16)).unwrap();
            assert_eq!(al, 0x14, "Safe-Op, with the error bit");
        }
        let mut states = [0u16; DRIVES];
        cycles(master, &group, 1, |i, inputs, _| states[i] = status(inputs).0);
        assert_eq!(states, [0x40; DRIVES], "out of Operational, each drive turns its torque off");
    });
}

/* ── the mailbox ── */

#[test]
fn coe_the_drives_say_what_they_are() {
    sandboxed("coe_the_drives_say_what_they_are", |master| {
        let group = init(master);
        for (i, d) in group.iter(master).enumerate() {
            let read = |index, sub| smol::block_on(d.sdo_read::<u32>(index, sub)).unwrap();
            assert_eq!(read(0x1000, 0), 0x0002_0192, "device type: CiA 402, a servo drive");
            assert_eq!(read(0x1018, 4), i as u32 + 1, "its serial number");
            assert_eq!(read(0x6502, 0), 0x0380, "supported modes: CSP, CSV, CST");
            assert_eq!(read(0x608F, 1), 1 << 17, "encoder counts a turn");
        }
        let rated: Vec<u32> = group.iter(master).map(|d| smol::block_on(d.sdo_read(0x6076, 0)).unwrap()).collect();
        assert_eq!(rated, [10_000, 10_000, 5_000], "rated torque, mN m, as the robot file says");
    });
}

#[test]
fn coe_bad_transfers_are_aborted_with_cia_301_codes() {
    sandboxed("coe_bad_transfers_are_aborted_with_cia_301_codes", |master| {
        let group = init(master);
        let d = group.iter(master).next().unwrap();
        assert_eq!(aborted(smol::block_on(d.sdo_read::<u32>(0x2000, 0))), CoeAbortCode::NotFound);
        assert_eq!(aborted(smol::block_on(d.sdo_read::<u32>(0x1018, 9))), CoeAbortCode::SubIndexNotFound);
        assert_eq!(aborted(smol::block_on(d.sdo_write(0x6041, 0, 0u16))), CoeAbortCode::ReadOnlyWrite);
        // The controlword is an output: it can't be mapped into a TxPDO.
        smol::block_on(d.sdo_write(0x1A00, 0, 0u8)).unwrap();
        assert_eq!(aborted(smol::block_on(d.sdo_write(0x1A00, 1, 0x6040_0010u32))), CoeAbortCode::PdoMappingFailed);
    });
}

#[test]
fn coe_pdos_remapped_in_pre_op_are_what_the_master_exchanges() {
    sandboxed("coe_pdos_remapped_in_pre_op_are_what_the_master_exchanges", |master| {
        let group = init(master);
        let d = group.iter(master).next().unwrap();
        // The first drive's TxPDO down to its statusword and position.
        smol::block_on(async {
            d.sdo_write(0x1C13, 0, 0u8).await?;
            d.sdo_write(0x1A00, 0, 0u8).await?;
            d.sdo_write(0x1A00, 1, 0x6041_0010u32).await?;
            d.sdo_write(0x1A00, 2, 0x6064_0020u32).await?;
            d.sdo_write(0x1A00, 0, 2u8).await?;
            d.sdo_write(0x1C13, 0, 1u8).await
        })
        .expect("the remapping");
        let group = smol::block_on(group.into_op(master)).expect("Operational");
        let sizes: Vec<usize> = group.iter(master).map(|d| d.io_raw().inputs().len()).collect();
        assert_eq!(sizes, [6, 15, 15]);
        let d = group.iter(master).next().unwrap();
        assert_eq!(aborted(smol::block_on(d.sdo_write(0x1A00, 0, 1u8))), CoeAbortCode::InvalidState, "only in Pre-Op");
        let mut state = 0;
        cycles(master, &group, 1, |i, inputs, _| {
            if i == 0 {
                state = status(inputs).0;
            }
        });
        assert_eq!(state, 0x40, "its statusword where the new mapping puts it: Switch On Disabled");
    });
}

/* ── CiA 402 ── */

#[test]
fn cia402_csp_moves_the_shoulder_where_the_master_says() {
    sandboxed("cia402_csp_moves_the_shoulder_where_the_master_says", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        enable(master, &group);
        let mut start = [0i32; DRIVES];
        cycles(master, &group, 1, |i, inputs, _| start[i] = status(inputs).1);
        // 0.5 rad on the shoulder, ramped over a second: 2^17 counts a turn.
        let goal = (0.5 / std::f64::consts::TAU * (1 << 17) as f64) as i32;
        let mut k = 0;
        cycles(master, &group, 1500, |i, inputs, out| {
            assert_eq!(status(inputs).0, 0x27, "drive {i} stays enabled");
            if i == 1 {
                let target = start[1] + (goal as f64 * (k as f64 / 1000.0).min(1.0)) as i32;
                out[2..6].copy_from_slice(&target.to_le_bytes());
                k += 1;
            }
        });
        let mut actual = [0i32; DRIVES];
        cycles(master, &group, 1, |i, inputs, _| actual[i] = status(inputs).1);
        let moved = (actual[1] - start[1]) as f64 / goal as f64;
        eprintln!("shoulder: from {} to {} counts, {:.0}% of the way to +0.5 rad", start[1], actual[1], moved * 100.0);
        assert!((0.9..1.1).contains(&moved), "the shoulder went {:.0}% of the way", moved * 100.0);
    });
}

#[test]
fn cia402_csv_turns_the_base_at_the_speed_asked() {
    sandboxed("cia402_csv_turns_the_base_at_the_speed_asked", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        enable(master, &group);
        // 1 rad/s, in counts a second.
        let target = (TURN / std::f64::consts::TAU) as i32;
        let mut speed = 0;
        cycles(master, &group, 500, |i, inputs, out| {
            if i == 0 {
                out[6..10].copy_from_slice(&target.to_le_bytes());
                out[12] = 9;
                speed = velocity_torque(inputs).0;
            }
        });
        let ratio = speed as f64 / target as f64;
        eprintln!("base: {speed} counts/s, {:.1}% of the {target} asked", ratio * 100.0);
        assert!((0.95..1.02).contains(&ratio), "the base turned at {:.0}% of the speed asked", ratio * 100.0);
    });
}

#[test]
fn cia402_cst_gives_the_torque_asked_up_to_its_max_torque() {
    sandboxed("cia402_cst_gives_the_torque_asked_up_to_its_max_torque", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        enable(master, &group);
        let cst = |torque: i16| {
            let mut seen = (0, 0);
            cycles(master, &group, 50, |i, inputs, out| {
                if i == 0 {
                    out[10..12].copy_from_slice(&torque.to_le_bytes());
                    out[12] = 10;
                    seen = velocity_torque(inputs);
                }
            });
            seen
        };
        // 100 ‰ of the base's rated 10 N m.
        let (speed, torque) = cst(100);
        assert_eq!(torque, 100, "the actual torque is the target's");
        assert!(speed > 0, "the base turns the way it's pushed");
        let d = group.iter(master).next().unwrap();
        smol::block_on(d.sdo_write(0x6072, 0, 50u16)).unwrap();
        assert_eq!(cst(-100).1, -50, "the max torque (0x6072) caps it");
    });
}

#[test]
fn cia402_quick_stop_brakes_then_switches_off() {
    sandboxed("cia402_quick_stop_brakes_then_switches_off", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        enable(master, &group);
        let target = (TURN / std::f64::consts::TAU) as i32;
        cycles(master, &group, 300, |i, _, out| {
            if i == 0 {
                out[6..10].copy_from_slice(&target.to_le_bytes());
                out[12] = 9;
            }
        });
        let mut states = Vec::new();
        cycles(master, &group, 500, |i, inputs, out| {
            if i == 0 {
                // Enable voltage, quick stop (bit 2) cleared.
                out[0..2].copy_from_slice(&0x0Bu16.to_le_bytes());
                let s = status(inputs).0;
                if states.last() != Some(&s) {
                    states.push(s);
                }
            }
        });
        // Operation Enabled, Quick Stop Active while it brakes, Switch On Disabled.
        assert_eq!(states, [0x27, 0x07, 0x40]);
    });
}

#[test]
fn cia402_a_following_error_faults_until_reset() {
    sandboxed("cia402_a_following_error_faults_until_reset", |master| {
        let group = smol::block_on(init(master).into_op(master)).expect("Operational");
        enable(master, &group);
        let mut start = 0;
        cycles(master, &group, 1, |i, inputs, _| {
            if i == 0 {
                start = status(inputs).1;
            }
        });
        // Ten turns away at once: outside the window (a turn) for longer
        // than its timeout (10 ms).
        let far = start + 10 * TURN as i32;
        let (mut state, mut code) = (0, 0);
        cycles(master, &group, 50, |i, inputs, out| {
            if i == 0 {
                out[2..6].copy_from_slice(&far.to_le_bytes());
                state = status(inputs).0;
                code = u16::from_le_bytes([inputs[13], inputs[14]]);
            }
        });
        assert_eq!((state, code), (0x08, 0x8611), "Fault, with the following error's code in 0x603F");
        cycles(master, &group, 5, |i, inputs, out| {
            if i == 0 {
                out[0..2].copy_from_slice(&0x80u16.to_le_bytes());
                state = status(inputs).0;
                code = u16::from_le_bytes([inputs[13], inputs[14]]);
            }
        });
        assert_eq!((state, code), (0x40, 0), "a fault reset: Switch On Disabled, the error cleared");
    });
}

/* ── not there yet ── */

#[test]
#[ignore = "not simulated: the chip's feature register says it has no distributed clocks, so masters don't set up SYNC0 or the system time"]
fn bus_distributed_clocks() {}

#[test]
#[ignore = "not simulated: wire time; a frame comes back as soon as drydock has processed it, after the veth pair's own tens of microseconds, not the under a microsecond a chip takes per device"]
fn bus_wire_time() {}

#[test]
#[ignore = "not simulated: FMMUs and PDO entries map whole bytes only, not single bits"]
fn coe_bit_sized_mappings() {}

#[test]
#[ignore = "not simulated: segmented SDO transfers; one longer than a mailbox (112 bytes of data) is refused"]
fn coe_segmented_transfers() {}

#[test]
#[ignore = "not simulated: SDO information; asking a drive to list its dictionary gets no answer"]
fn coe_sdo_information() {}

#[test]
#[ignore = "not simulated: emergencies; a fault shows in the statusword and 0x603F only"]
fn coe_emergencies() {}

#[test]
#[ignore = "not simulated: any one real drive; its identity is the robot file's, and its objects the ones CiA 402 and CSP, CSV and CST need"]
fn cia402_a_real_drive_s_own_objects() {}
