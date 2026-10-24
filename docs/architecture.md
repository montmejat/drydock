# drydock's design

drydock puts a simulated robot behind real bus interfaces. A bus master (a
robot's driver, its ROS 2 hardware interface, keel) opens `vcan0` instead of
`can0`, sends the frames it would send to the motors, and gets the answers
the motors would give, when they would give them, while MuJoCo moves the
robot.

This page is how it's built, why, and what's missing. The README is how to
use it.

## 1. What is real, what is modelled, what is absent

The line is drawn at the bus, on purpose: everything above it is the robot's
own software, unchanged.

| | |
|---|---|
| **Real**: the same code as on the robot | the bus master and its driver library, its socket calls, its threads and timing on the host, its handling of every answer |
| **Modelled**: real code paths, simulated input | each motor's firmware (answers, states, registers, timeout, MIT control law), its torque limit and transmission, its encoder's errors, its faults on a flag, the bus's timing (frame lengths, arbitration), the robot's mechanics (MuJoCo, from the robot's own MJCF) |
| **Absent**: not exercised at all | the bus's electrical faults (error frames, bus-off), the motors' other modes, thermals, power |

Most simulators draw the line higher: the robot's software talks to the
simulator through an adapter of its own (a Python API, a TCP protocol), and
the driver never runs off the robot. drydock's job is the part they leave
out.

## 2. The pieces

```
 bus masters (wave, openarm_can, keel…), one per bus
        │  CAN frames on vcan0, vcan1…
 ┌──────▼─────────────────────────────────────────────────────────────┐
 │ can/          frames, SocketCAN                                    │
 │ can/timing    how long each frame takes, which goes first          │
 │ run           the clock: real time or lockstep, over every bus     │
 │ dock          the robot: buses of motors on the world's joints     │
 │                                                                    │
 │  damiao/bus ── damiao/motor ── drivetrain ────────── world         │
 │  whose frame   the firmware:    actuator (peak       MuJoCo, via   │
 │  it is         answers, the     torque) and          shim/world.c  │
 │                MIT law          transmission                       │
 └────────────────────────────────────────────────────────────────────┘
   robot: the robot file, which bus, which motor, which joint
```

| Piece | What it knows | What it doesn't |
|---|---|---|
| `can` | frames (classic, CAN FD), a socket per interface, waiting on several | protocols |
| `can::timing` | a frame's bits, stuffing included, at its bus's rates; one bus's queue, in arbitration order | what frames mean |
| `run` | the clock; which frame crosses when; answers back to the master | the protocol, the physics |
| `damiao::protocol` | Damiao's frames: commands, feedback, special frames, registers, packing, the motors' factory ranges | anything but bytes ↔ values |
| `damiao::motor` | one motor's firmware: disabled, enabled or in error; its command; its registers; its timeout; the torque its law asks for | what limits that torque, joints, MuJoCo: it's handed `(t, θ, θ̇)` and the torque it got |
| `damiao::bus` | the motors on one bus: whose a frame is, CAN FD or classic, notes on what's ignored | timing, physics |
| `drivetrain` | what a motor can give (peak torque), how its turning moves its joint (transmission), what it senses of it | protocols |
| `effects` | an encoder's errors (delay, bias, drift, noise, counts), faults (dead, weak, frozen, noisy), a seeded generator | motors, protocols |
| `world`, `shim/world.c` | MuJoCo: joints by name, their state, forces on them, stepping, keyframes, silencing the scene's own actuators | motors |
| `dock` | the robot: each bus's motors, each motor's drivetrain and joint | sockets, clocks |
| `robot` | the robot file | |

`damiao::protocol` serves both sides: the `wave` example uses it to talk to
drydock as a master would.

A second protocol (Dynamixel, for Microduck) would be a sibling of `damiao`
(its frames, its devices, which device a packet is for) on a sibling of
`can` (a pseudo-terminal); `drivetrain`, `world`, `dock` and `run` would
serve it as they are, with a sensor next to the drivetrain for its IMU.

## 3. A frame's way through, in real time

1. `run` reads a frame from a bus's socket, classic or CAN FD, and queues
   it on that bus's timeline, ready now.
2. When the bus is free, the waiting frame with the lowest id starts
   across: arbitration. It takes its bits' time: the nominal rate for its
   header and tail, the data rate for a CAN FD frame's data.
3. When it's across, `dock` delivers it: `damiao::bus` finds whose it is (a
   motor's id; `0x7FF` and the id inside; another mode's id, noted and
   ignored; a CAN FD frame to motors on classic CAN, noted and ignored).
4. The motor reads its angle and velocity through its transmission, acts on
   the frame (enable, a new command, a register write…) and answers:
   feedback on its master id, or a register's value.
5. The answer is queued on the same timeline, ready `reply_us` later, and
   crosses in turn; when it's across, `run` sends it to the master.

The master's write returns at once, as on a real bus (the frame waits in
its CAN controller); what takes time is everything after it.

## 4. A step's way through

1. For each motor: its angle and velocity, through its transmission, then
   its encoder (late, offset, noisy, counted, or frozen).
2. The motor's law asks for a torque: `kp (p_des − θ) + kd (v_des − θ̇) +
   t_ff` on its latest command if it's enabled, none otherwise. A motor past
   its timeout goes to error `D` here.
3. The actuator gives what it can (within the peak torque, none if it's
   dead, a fifth if it's weak); the motor records it, to report.
4. Through the transmission, it's the force on the joint (MuJoCo's
   `qfrc_applied`).
5. MuJoCo steps once, by the scene's timestep.

The law runs every physics step, on the latest command: a real Damiao drive
runs its loop at tens of kHz, so holding a command between frames is the
drive's behaviour, not a simplification of it.

## 5. Time

**Real time** (default): a physics step each timestep of the wall clock,
and frames crossing their buses as above, in between. Falling more than
100 ms behind is said and forgiven, not caught up. A bus kept busy (90% of
a second, or a frame waiting a millisecond) says so, once a second.

**Lockstep** (`--lockstep <hz>`): physics advances `1/hz` seconds once every
motor, on every bus, has had a frame since the last advance. Frames take no
time; answers go out at once, with the state before the advance, so a
master sending each motor one command per cycle reads, a cycle later, what
its commands did. Runs repeat frame for frame, and go as fast as the
masters.

The rule ("every motor has heard from its master") needs no clock protocol
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

**The robot file is what drives it**: its buses, each with its interface,
CAN FD or not, bit rates, and motors; for each motor, its joint, model, ids,
and what differs from the factory (ranges, peak torque, timeout,
transmission). Ids are per bus: two arms on two buses use the same ones, as
OpenArm's do.

This split is what makes a published model usable as it is, and keeps the
motors' behaviour out of the scene, where it would have to be faked with
MuJoCo actuators.

## 7. Decisions

**Rust, MuJoCo's C library through a C shim.** The shim (90 lines of C) is
the only code that touches MuJoCo's structs, compiled against its headers,
so Rust mirrors no struct layout and a MuJoCo upgrade can't silently
misread memory. MuJoCo is a shared library found at build time and linked
with an rpath.

**MuJoCo's C library, not MJX or MuJoCo Warp.** One world, stepped as the
buses ask, at a 1 ms timestep: a CPU's job. The GPU versions read the same
MJCF, so models move between them.

**One protocol, completely.** Damiao's MIT mode, frame for frame against
Damiao's reference driver and OpenArm's library. A second protocol half done
would make neither trustworthy.

**The firmware and the physics apart.** A motor's firmware asks for a
torque; its actuator gives what it can; the motor reports what it got, as a
real drive reports its measured current. Limits, and later noise, delay and
faults, live in the drivetrain, not in the protocol's code.

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

**Frames are counted bit by bit.** A frame's length is computed from its
actual bits, as ISO 11898-1 lays them out: the CRC-15 of a classic frame
(stuffing covers it), CAN FD's fixed stuff bits around its CRC, and dynamic
stuffing over the rest. A frame's length depends on its content by up to
20%; an average would hide when the bus is close to full.

**The master's frames compete with the motors' answers.** All frames waiting
for the bus compete when it frees, the lowest id first: a master that queues
its eight commands at once sends all eight before the first answer (at
higher ids) goes. That is what a CAN controller with several transmit
buffers does; one that sends a frame at a time, with gaps between, would let
answers in between.

**Faults are on a flag, and say nothing.** A motor breaks when the robot
file or `--break` says, in simulated seconds, so a run that finds it
repeats. A broken motor still reports itself enabled: a dead drive or a
frozen encoder doesn't announce itself on a real robot either, and noticing
is the bus master's job, or its health checks'. Run against keel with the
elbow dying at 2 s, the elbow drops to its stop within a second, and nothing
in the logs says so.

**Lockstep has no timing.** It's for repeatable, faster-than-real-time runs;
frames taking time would only add delays the master can't see in its
stamps.

## 8. What OpenArm taught it

Running OpenArm's own library against drydock found:

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
  as motor state: a position near −12.5 rad. With bus timing, answers come
  ~50 µs (CAN FD) to ~120 µs (classic) apart, as on a real bus, so it would
  happen there too. `hold` lets answers settle during setup.
- **Classic CAN is full at 500 Hz.** Measured with bus timing, `hold`'s
  cycle (8 commands, 8 answers, every 2 ms) keeps a classic bus 96% busy,
  with frames waiting up to 8.8 ms while the queue drains at start; on CAN
  FD, 41%. The frame formats predicted 90–110% and 41–45%.

The third is the kind of thing drydock is for: a bug in a driver, found
without a robot. The fourth: a bus too slow for its loop, found without
one.

## 9. What's next

**A second protocol.** Dynamixel 2.0 on a serial line, for Microduck: as in
section 2.

**Two arms.** OpenArm's bimanual model on two buses: the robot file
supports it; an example doesn't exist yet.

**A test of OpenArm.** Today it's run by hand. A test needs the model, which
`setup.sh` fetches; an ignored test, or one that skips without it, run in CI
after `setup.sh`.

**Seeing it.** MuJoCo's viewer, or a recording of states to replay later;
today drydock prints nothing while it runs, but for a busy bus.
