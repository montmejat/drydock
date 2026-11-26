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
use ethercrab::std::{ethercat_now, tx_rx_task};
use ethercrab::subdevice_group::{Op, PreOp, SubDeviceGroup};
use ethercrab::{MainDevice, MainDeviceConfig, PduStorage, SubDeviceState, Timeouts};

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
    std::thread::spawn(|| {
        let robot = Robot::load(concat!(env!("CARGO_MANIFEST_DIR"), "/examples/ethercat-arm/robot.yml")).unwrap();
        let dock = Drydock::load(&robot).unwrap();
        let line = EthercatLine { name: "ecs0".into(), socket: Socket::open("ecs0").unwrap() };
        run::realtime(dock, &[], &mut [], &[line], None)
    });

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
