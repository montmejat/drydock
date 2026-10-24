# drydock's design

drydock puts a simulated robot behind a real bus interface. A bus master (a
robot's driver, its ROS 2 hardware interface, keel) opens `vcan0` instead of
`can0`, sends the frames it would send to the motors, and gets the answers
the motors would give, while MuJoCo moves the robot.

This page is how it's built, why, and what's missing. The README is how to
use it.

## 1. What is real, what is modelled, what is absent

The line is drawn at the bus, on purpose: everything above it is the robot's
own software, unchanged.

| | |
|---|---|
| **Real**: the same code as on the robot | the bus master and its driver library, its socket calls, its threads and timing on the host, its handling of every answer |
| **Modelled**: real code paths, simulated input | each motor's protocol behaviour (answers, states, registers, timeout), its MIT control law and torque limit, the transmission to its joint, the robot's mechanics (MuJoCo, from the robot's own MJCF) |
| **Absent**: not exercised at all | the bus's electrical timing and arbitration, the motors' other modes, thermals, power, faults other than lost communication |

Most simulators draw the line higher: the robot's software talks to the
simulator through an adapter of its own (a Python API, a TCP protocol), and
the driver never runs off the robot. drydock's job is the part they leave
out.

## 2. The pieces

```
 bus master (wave, openarm_can, keel…)
        │  CAN frames on vcan0
 ┌──────▼───────────────────────────────────────────────────────┐
 │ can.rs     transport: a SocketCAN socket                      │
 │ run.rs     clock: real time, or lockstep with the master      │
 │ dock.rs    the robot: routes frames to motors, steps physics  │
 │                                                               │
 │  motor.rs        angle, velocity              world.rs        │
 │  one Damiao  ◄──────────────── Transmission ◄──── MuJoCo, via │
 │  motor       ────────────────► (robot.rs) ─────► shim/world.c │
 │  (damiao.rs)      torque                       joint force    │
 └───────────────────────────────────────────────────────────────┘
   robot.rs: the robot file, which motor drives which joint
```

| File | What it knows | What it doesn't |
|---|---|---|
| `damiao.rs` | Damiao's frames: commands, feedback, special frames, registers, packing, the motors' factory ranges | anything but bytes ↔ values: no state, no I/O, no physics |
| `motor.rs` | one motor's behaviour: disabled, enabled or in error; its command; its registers; its timeout; the MIT law and torque limit | MuJoCo, joints, transmissions: it's handed `(t, θ, θ̇)` and gives `τ` |
| `robot.rs` | the robot file; `Transmission`, the linear map between a motor and its joint | |
| `world.rs`, `shim/world.c` | MuJoCo: joints by name, their state, forces on them, stepping, keyframes, silencing the scene's own actuators | motors and protocols |
| `dock.rs` | the robot: which motor is on which joint, through which transmission; frames to motors; torques to joints | sockets and clocks |
| `run.rs` | how time passes | the protocol |
| `can.rs` | SocketCAN, classic and CAN FD frames | |

`damiao.rs` serves both sides: the `wave` example uses it to talk to drydock
as a master would.

## 3. A frame's way through

1. `run.rs` reads a frame from the socket, classic or CAN FD.
2. `dock.rs` ignores a CAN FD frame if the motors are on classic CAN (a
   real bus would carry error frames), then finds whose it is: a frame to a motor's id goes to that motor;
   a frame to `0x7FF` goes to the motor named in its first two bytes; a
   frame to `id + 0x100/0x200/0x300` (another mode) is ignored, with a note
   once.
3. The motor's angle and velocity are read from its joint, through its
   transmission.
4. The motor acts on the frame (enable, a new command, a register write…)
   and answers: feedback on its master id, or a register's value.
5. `run.rs` sends the answer at once, in CAN FD if the motors are on it.

Answers carry the state of that moment, as a real motor's do.

## 4. A step's way through

1. For each motor: its angle and velocity, through its transmission.
2. The motor gives a torque: the MIT law on its latest command,
   `kp (p_des − θ) + kd (v_des − θ̇) + t_ff`, within its peak torque, if it
   is enabled; none otherwise. A motor past its timeout goes to error `D`
   here.
3. The torque, through the transmission, becomes the force on the joint
   (MuJoCo's `qfrc_applied`).
4. MuJoCo steps once, by the scene's timestep.

The law runs every physics step, on the latest command: a real Damiao drive
runs its loop at tens of kHz, so holding a command between frames is the
drive's behaviour, not a simplification of it.

## 5. Time

**Real time** (default): a physics step each timestep of the wall clock;
frames answered between steps, as they arrive. Falling more than 100 ms
behind is said and forgiven, not caught up.

**Lockstep** (`--lockstep <hz>`): physics advances `1/hz` seconds once every
motor has had a frame since the last advance. Answers go out at once, with
the state before the advance, so a master sending each motor one command per
cycle reads, a cycle later, what its commands did. Runs repeat frame for
frame, and go as fast as the master.

The rule ("every motor has heard from the master") needs no clock protocol
on the bus: CANopen's SYNC would have been the alternative, but Damiao's
motors don't speak it, and a master shouldn't have to change to be
simulated. Its cost: a master that leaves a motor out of its cycles stalls
time.

## 6. A robot: two files

**The scene (MJCF) is the mechanics**: links, masses, inertias, joint
limits, rotor inertia (`armature`), damping, friction. It's the robot's own
model, unchanged: OpenArm's comes from `openarm_mujoco` as Enactic publishes
it. Its actuators on driven joints are silenced (zero gain and bias), since
drydock's motors drive those joints now.

**The robot file is what drives it**: for each motor, its joint, model,
ids, and what differs from the factory (ranges, peak torque, timeout,
transmission).

This split is what makes a published model usable as it is, and keeps the
motor's behaviour out of the scene, where it would have to be faked with
MuJoCo actuators.

## 7. Decisions

**Rust, MuJoCo's C library through a C shim.** The shim (90 lines of C) is
the only code that touches MuJoCo's structs, compiled against its headers,
so Rust mirrors no struct layout and a MuJoCo upgrade can't silently
misread memory. MuJoCo is a shared library found at build time and linked
with an rpath.

**MuJoCo's C library, not MJX or MuJoCo Warp.** One world, stepped as the
bus asks, at a 1 ms timestep: a CPU's job. The GPU versions read the same
MJCF, so models move between them.

**One protocol, completely.** Damiao's MIT mode, frame for frame against
Damiao's reference driver and OpenArm's library. A second protocol half done
would make neither trustworthy.

**Unknown registers get no answer.** drydock answers the registers it
simulates (`MST_ID`, `ESC_ID`, `CTRL_MODE`, `PMAX`, `VMAX`, `TMAX`) and
stays silent on the others, saying so once. Answering a register it doesn't
model with a made-up value would let a driver believe something false; a
missing answer is a failure the driver already handles.

**Refused writes answer with the kept value.** Writing another mode to
`CTRL_MODE`, or new ids, is answered with the value the motor keeps, which
is how a driver that checks its writes (both reference drivers do) finds out.

**Torque goes in as an applied force.** `qfrc_applied` on the joint's degree
of freedom, rather than a MuJoCo actuator, so a scene needs no edits and the
motor's law lives in one place.

**The transmission is linear.** OpenArm's driver maps its gripper motor to
finger travel linearly; drydock does the same, so the simulated gripper is
where the driver thinks it is. A nonlinear linkage would be modelled in the
scene, as joints.

**CAN FD is the motors' setting, in the robot file.** On a real robot,
whether the motors are on CAN FD is how they're configured, not something
each frame decides. Motors on CAN FD answer in it (8 bytes, data at the
faster bit rate, as `openarm_can` sends) and understand classic frames, as
CAN FD controllers do; motors on classic CAN ignore CAN FD frames and say
so. A master set to the wrong kind finds out the way it would on the robot:
silence. The Damiao frames are the same in both.

**Answers go out at once.** No bus timing: an answer leaves within
microseconds of its frame (8 register reads answered within 64 µs, measured).
A real 1 Mbit/s bus spaces classic frames about 0.1 ms apart.

## 8. What OpenArm taught it

Running OpenArm's own library against drydock found three things:

- **The two Damiao drivers disagree on a range.** OpenArm's packs a DM4340's
  velocity into ±10 rad/s; Damiao's reference driver says 8. It's a
  register, set per motor, so the robot file now says so (`limits`), and
  OpenArm's robot file follows OpenArm.
- **The gripper needs a transmission.** Its motor turns; its fingers slide.
  Hence `Transmission`. A check that it's right: the DM4310's 7 N m, at
  0.042 m/rad, is 167 N at the fingers, near the 150 N OpenArm's model
  allows them.
- **`openarm_can`'s `recv_all` can read an answer in the wrong mode.** It
  waits for the first answer, then takes only what is already there. A
  register answer arriving a little later is read on the next call, perhaps
  as motor state: a position near −12.5 rad. Measured here with eight
  register reads in a row; on a real bus, with answers 0.1 ms apart, it
  plausibly happens too (not verified on hardware). `hold` lets answers
  settle during setup.

The last is the kind of thing drydock is for: a bug in a driver, found
without a robot.

## 9. What's next

**Building blocks** (before a second protocol). `motor.rs` holds two things
that vary apart: the drive's logic (what it answers, its states, its
registers: the protocol's) and the actuator's physics (the law, the limits,
later noise, delay and faults). `dock.rs` is wired to one CAN bus of Damiao
motors. Making them explicit:

1. transport (a CAN socket, a pty for serial buses);
2. codec (Damiao; Dynamixel next): bytes ↔ messages;
3. device: a motor or sensor as the bus sees it;
4. actuator model: a command and `(θ, θ̇)` to a torque;
5. sensor model: the world to a reading (an IMU);
6. world: MuJoCo;
7. clock: real time or lockstep, over any set of transports.

Then the robot file lists buses, each with its devices, and Microduck
(Dynamixel servos and an IMU on a serial bus) is new files, not changes to
the Damiao ones.

**Bus timing.** Space answers as the bus would: frame length at the bit
rate, and a drive's reply delay. Without it, drivers that depend on answers
arriving later (as `recv_all` does) look better than they are, and a bus
that's too slow for its loop doesn't show. The numbers, from the frame
formats (ISO 11898-1), for a Damiao frame of 8 bytes:

| | classic, 1 Mbit/s | CAN FD, 1 Mbit/s, data at 5 Mbit/s |
|---|---|---|
| one frame | 111–135 bits: 111–135 µs | ~30 bits at 1 M and 96–113 at 5 M: ~49–57 µs |
| OpenArm's cycle: 8 commands, 8 answers | 1.8–2.2 ms | 0.8–0.9 ms |
| bus load at 1 kHz | 180–220%: doesn't fit | 80–90% |

(Ranges: no bit stuffing to the worst case.) The robot file would gain the
bit rates; the bus would be one queue, frames leaving it in id order, as
arbitration orders them.

**Hardware effects.** Encoder noise and quantization, command delay, and
faults on a flag (a dead motor, a frozen encoder), seeded, so health checks
have something to find.

**A test of OpenArm.** Today it's run by hand. A test needs the model, which
`setup.sh` fetches; an ignored test, or one that skips without it, run in CI
after `setup.sh`.

**Seeing it.** MuJoCo's viewer, or a recording of states to replay later;
today drydock prints nothing while it runs.
