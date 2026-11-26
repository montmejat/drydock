# EtherCAT

drydock simulates CiA 402 servo drives on an EtherCAT line, each on its own
EtherCAT chip (a Beckhoff ET1100). A real EtherCAT master, such as SOEM,
ethercrab or IgH, opens one end of a virtual Ethernet cable. drydock sits on
the other end and plays every drive on it. MuJoCo moves the joints.

The example is a three-joint arm, [`examples/ethercat-arm`](../examples/ethercat-arm):
a base that turns, a shoulder and an elbow, one drive each, on one cable.

## The chain, end to end

One cycle, at 1 kHz for a robot arm, from the master's control loop to the
motors and back:

```
 master (SOEM, ethercrab)                         drydock
 ┌────────────────────┐   one Ethernet frame   ┌──────────┐  ┌──────────┐  ┌──────────┐
 │ control loop       │ ─────────────────────► │ drive 0  │─►│ drive 1  │─►│ drive 2  │─┐
 │ process image:     │                        │ ET1100   │  │ ET1100   │  │ ET1100   │ │ port 1 open:
 │ every drive's      │ ◄───────────────────── │ firmware │◄─│ firmware │◄─│ firmware │◄┘ the frame turns
 │ outputs and inputs │   the same frame back  └────┬─────┘  └────┬─────┘  └────┬─────┘
 └────────────────────┘                             │ torque      │             │
                                                    ▼             ▼             ▼
                                                  MuJoCo: base, shoulder, elbow
```

1. **The control loop** decides each joint's target: a position in CSP, a
   velocity in CSV, a torque in CST.
2. **The master** keeps the *process image*: one block of bytes with every
   drive's outputs (controlword, targets, mode) and inputs (statusword,
   actual position, velocity, torque). Each cycle it sends that block in
   one frame, as one LRW datagram (logical read-write).
3. **Each drive's chip** processes the frame as it passes, in hardware. Its
   *FMMU* maps a slice of the master's logical addresses to its own memory:
   it takes its outputs from the frame and puts its inputs in their place.
   It adds to the datagram's *working counter* (1 for a read, 2 for a
   write) and passes the frame on.
4. **The last drive** has nothing on its second port, so the frame turns
   back there. It passes back through every drive to the master.
5. **The master checks the working counter.** Three drives that each read
   and write give 9; anything less means one didn't answer.
6. **Each drive's firmware** gets its outputs from the chip's *sync
   manager*, a buffer the chip hands over only once the master has written
   all of it. It runs CiA 402's state machine on the controlword, then its
   loops on the target. The torque reaches the joint, and the encoder's
   reading goes back into the inputs for the next frame.

### Before that: bringing the line up

A master walks every device through EtherCAT's states (the *AL states*),
and each step needs more set up:

| State | What the master does there | What drydock's drives check |
|---|---|---|
| **Init** | counts the devices (a broadcast read), gives each a station address (by position), reads each one's EEPROM (SII): identity, mailbox, sync managers, default PDOs | |
| **Pre-Operational** | the mailbox works: configures the drive over CoE (SDOs): its PDO mapping (0x1600, 0x1A00, 0x1C12, 0x1C13), its mode (0x6060), its limits | the mailbox sync managers are where the EEPROM says, or `0x0016` |
| **Safe-Operational** | sets the process data sync managers and FMMUs; inputs flow, outputs don't act yet | the sync managers match the PDOs assigned, or `0x001D` (outputs) / `0x001E` (inputs) |
| **Operational** | outputs act: the cyclic exchange above | outputs keep coming, or the watchdog (100 ms) drops the drive to Safe-Op with `0x001B` |

A refused step leaves the drive where it was, with the error bit set in AL
Status (0x0130) and the reason in AL Status Code (0x0134), until the master
acknowledges it.

Then CiA 402 enables the drive, by the controlword:

| Controlword | Transition | Statusword after |
|---|---|---|
| (power on) | | `0x0040` Switch On Disabled |
| `0x06` Shutdown | 2 | `0x0021` Ready To Switch On |
| `0x07` Switch On | 3 | `0x0023` Switched On |
| `0x0F` Enable Operation | 4 | `0x0027` Operation Enabled |

In CSP, copy the actual position (0x6064) into the target (0x607A) before
enabling. Otherwise the drive jumps to whatever the target held. If the
following error (0x60F4) stays outside its window (0x6065) longer than its
timeout (0x6066), the drive faults with error code `0x8611`. A rising edge
on controlword bit 7 resets a fault.

## Running it

The master and drydock each need raw sockets on their end of a veth pair.
A network namespace of your own gives both, without root:

```sh
cargo build --release
unshare -rn sh -c '
  ip link add ecm0 type veth peer name ecs0 && ip link set ecm0 up && ip link set ecs0 up
  ./target/release/drydock examples/ethercat-arm/robot.yml &
  sleep 1
  slaveinfo ecm0 -map        # SOEM'"'"'s, or any master on ecm0
'
```

Or, on the host, once per boot, with the master and drydock allowed raw
sockets:

```sh
sudo ip link add ecm0 type veth peer name ecs0 && sudo ip link set ecm0 up && sudo ip link set ecs0 up
sudo setcap cap_net_raw+ep target/release/drydock
```

SOEM's `slaveinfo ecm0 -map` lists the three drives with their sync
managers, FMMUs and PDO mapping, read over CoE, and brings them to
Safe-Operational. It takes 23 s: it asks each mapped object's name over
SDO information, which drydock doesn't have, and waits out SOEM's 0.7 s
mailbox timeout on each (`slaveinfo ecm0` alone takes 0.02 s). Its `simple_ng ecm0` takes them to Operational and
exchanges process data each cycle: a working counter of 9.

[`tests/ethercat.rs`](../tests/ethercat.rs) runs ethercrab against the arm
the same way, one test per feature (the AL states and the SM watchdog,
CoE and PDO remapping, CSP, CSV, CST, quick stop, the following error,
and what a scenario breaks: a drive disconnected, dead, shorted, a lossy
link, the supply off) and
an ignored test for each gap below. Each test runs itself in a network
namespace of its own, which needs unprivileged user namespaces: on Ubuntu
24.04, `sudo sysctl -w kernel.apparmor_restrict_unprivileged_userns=0`.

## The robot file

```yaml
ethercat:
  - interface: ecs0
    drives:                 # in the order the cable runs through them
      - { joint: base, rated_torque: 10, peak_torque: 30, gains: { kp: 300, kd: 10, kv: 5 } }
```

See [the robot file](robot-file.md#ethercat-lines).

## What's real, what's simplified

**The chip.** Its register map is an ET1100's (type `0x11`, 8 FMMUs, 8 sync
managers, 8 KB of RAM): the address modes (auto-increment, configured,
broadcast, logical), every read/write command and its working counter, the
EEPROM interface (8-byte reads), mailbox sync managers (read only when full,
written only when empty), the SM watchdog, DL Status showing which ports
are linked, and the source MAC bit it sets on the way back.

**The drive.** The AL state machine and its status codes; CoE SDO uploads
and downloads, expedited and normal, with complete access; PDO remapping in
Pre-Op, checked entry by entry; CiA 402's state machine, quick stop and
fault reset; CSP, CSV and CST; the following error.

**Not there yet:**

- **Distributed clocks.** The chip's feature register says it has none, so
  masters don't set them up. A real ET1100 has them; adding them is next.
- **Bit-sized mappings.** FMMUs and PDO entries map whole bytes only.
- **Segmented SDOs, SDO information, emergencies.** A transfer longer than
  one mailbox (112 bytes of data) is refused. Asking a drive to list its
  dictionary isn't answered (SOEM's `slaveinfo -sdo` prints none).
- **Wire time.** A frame comes back as soon as drydock has processed it. A
  real chip takes under a microsecond per device; the veth pair's own
  latency, tens of microseconds, is the only delay.
- **One real drive.** Its identity is the robot file's (`vendor`,
  `product`), zero by default: the drive plays no product yet.

**Where the sources are silent, drydock chooses:** the loops' gains (in
the robot file), the following error's defaults (a turn, for 10 ms), and
what a drive does when it leaves Operational: it turns its torque off
(Switch On Disabled). Some real drives fault instead.

## Sources

- Beckhoff, *EtherCAT Slave Controller, Section I: Technology*: frame
  processing, commands, the working counter.
- Beckhoff, *Section II: Register Description*: the register map, the EEPROM
  interface, sync managers, FMMUs, the watchdog.
- Beckhoff, *Section III: ET1100 Hardware Description*: its FMMU, sync
  manager and RAM counts.
- ETG.1000.6 (application layer), by way of the state handling, status
  codes and SII layouts of [SOEM](https://github.com/OpenEtherCATsociety/SOEM)
  and [ethercrab](https://github.com/ethercrab-rs/ethercrab).
- CiA 301 (the SDO protocol and its abort codes) and CiA 402 (IEC
  61800-7-201: the state machine, the controlword and statusword, the
  objects and units), by way of drive manuals.
