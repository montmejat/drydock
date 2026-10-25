# drydock

A robot's motors on a virtual CAN bus, their physics simulated in MuJoCo.

Software that drives real motors over SocketCAN drives drydock's the same
way: open `vcan0` instead of `can0`, and the frames it sends move a simulated
robot, which answers as the motors would. Nothing in the robot's software
knows it is simulated, so the driver and its timing are tested too, not
replaced.

drydock speaks one protocol, and tries to speak it completely: **Damiao's
motors in MIT mode** (DM-J4310, DM-J4340, DM-J8009 and the rest), as on
OpenArm and many other arms and legs.

## Install

Linux (x86-64), with Rust, a C compiler, `git` and `curl`. On Debian or
Ubuntu:

```sh
sudo apt install build-essential git curl
curl https://sh.rustup.rs -sSf | sh       # Rust, if you don't have it
```

MuJoCo 3.16.0, once, into `~/.cache/drydock` (or anywhere, with `MUJOCO_DIR`
pointing at it). The build says the same if it's missing:

```sh
mkdir -p ~/.cache/drydock && cd ~/.cache/drydock && \
  curl -fLO https://github.com/google-deepmind/mujoco/releases/download/3.16.0/mujoco-3.16.0-linux-x86_64.tar.gz && \
  echo "139577c541d43716a63992ab698cbc7b09788b1d547fb82b682d59b40ef49b9d  mujoco-3.16.0-linux-x86_64.tar.gz" | sha256sum -c && \
  tar xzf mujoco-3.16.0-linux-x86_64.tar.gz && rm mujoco-3.16.0-linux-x86_64.tar.gz
```

Then:

```sh
cargo install --path .                    # or: cargo build --release
```

A virtual CAN interface needs root, once per boot; drydock says so, with the
command, when it's missing:

```sh
sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 mtu 72 up
```

## Try it

```sh
drydock examples/arm/robot.yml
```

In another terminal, a bus master swings the example arm's two joints:

```sh
cargo run --release --example wave -- examples/arm/robot.yml
```

```
t =   1.0 s  shoulder: +0.538 rad -1.57 N m [1]  elbow: +0.010 rad -0.32 N m [1]
t =   1.5 s  shoulder: +0.396 rad -1.69 N m [1]  elbow: -0.494 rad -0.26 N m [1]
```

`cargo test` runs the protocol's and the motor's unit tests, and the arm in
MuJoCo frame by frame without a bus.

## OpenArm

[OpenArm](https://github.com/enactic/openarm) v1, one arm and its gripper,
on Enactic's own MuJoCo model, which drydock fetches on first run (the
robot file names its repository and commit):

```sh
drydock examples/openarm/robot.yml
```

[keel](https://github.com/montmejat/keel) drives it: `keel run
examples/openarm.yml` there. So can OpenArm's own CAN library, `openarm_can`,
with the motor types, ids and gains of its ROS 2 driver, through
`examples/openarm/hold.cpp`:

```sh
git clone -q --depth 1 --branch 1.4.1 https://github.com/enactic/openarm_can ~/.cache/drydock/openarm_can-1.4.1
c=~/.cache/drydock/openarm_can-1.4.1
g++ -std=c++17 -O2 -I $c/include examples/openarm/hold.cpp $c/src/openarm/*/*.cpp $c/src/openarm/can/socket/*.cpp \
  -o examples/openarm/hold
examples/openarm/hold            # in another terminal from drydock
```

```
master ids: 0x11 0x12 0x13 0x14 0x15 0x16 0x17 0x18
t =  2.5 s -0.26 -0.00 -0.00 +1.64 -0.00 -0.00 +0.06 | gripper -1.03 rad -0.10 N m
```

On CAN FD, as OpenArm's motors are by default (`hold --classic` tries a
classic master against them: it hears nothing, as on a real bus).

## Design

[docs/architecture.md](docs/architecture.md): the pieces, how a frame and a
step go through them, the decisions behind them and what's left to do.

## A robot

A robot is two files. The **scene** (MJCF) says how it moves: links, masses,
joints, rotor inertia (`armature`), damping, friction. The **robot file**
says what drives it: one or more CAN buses (two arms are often two buses),
each with its motors.

```yaml
scene: arm.xml              # relative to this file, or a published model, fetched once:
# scene: { git: https://github.com/enactic/openarm_mujoco, commit: a3c8a57…, file: v1/openarm.xml }
keyframe: home              # optional: where it starts
buses:
  - can: vcan0              # the interface
    fd: false               # optional: whether the motors are on CAN FD
    bitrate: 1000000        # optional: 1 Mbit/s by default
    data_bitrate: 5000000   # optional: CAN FD's data phase, 5 Mbit/s by default
    reply_us: 0             # optional: how long a motor takes to answer
    motors:
      - { joint: shoulder, model: DM4340, id: 0x01, master_id: 0x11 }
      - { joint: elbow, model: DM4310, id: 0x02, master_id: 0x12, peak_torque: 7, timeout_ms: 100 }
      - { joint: finger, model: DM4310, id: 0x08, master_id: 0x18, transmission: { ratio: 0.042, offset: 0.044 } }
      - { joint: wrist, model: DM4310, id: 0x03, master_id: 0x13, encoder: { noise: 0.0005, quantum: 0.0004 } }
seed: 0                     # optional: for the encoders' noise
```

The scene: a file next to the robot file, or a file in a repository at a
pinned commit (`git`, `commit`, `file`), which drydock fetches the first
time it runs, with `git`, into `~/.cache/drydock/models` (or
`$DRYDOCK_CACHE`): one commit deep, only the folder the file is in.

A bus:

- `fd`: the motors answer in CAN FD (8 bytes, data at the faster bit rate),
  and understand both kinds, as CAN FD controllers do. Motors on classic
  CAN ignore CAN FD frames, with a note: on a real bus they'd answer with
  error frames. The interface must carry CAN FD: its MTU 72 (drydock says
  how, if not).
- `bitrate`, `data_bitrate`: how fast frames cross the bus (see Time).
- `reply_us`: from a frame reaching a motor to its answer being ready to
  go. Unknown for Damiao's motors, so none by default.

A motor:

- `joint`: a hinge or slide joint in the scene. If the scene has its own
  actuators on it (MuJoCo Menagerie's models do), they're silenced: the
  motor drives the joint now.
- `model`: sets PMAX, VMAX, TMAX to the factory's, as Damiao's reference
  driver lists them. `limits: { p, v, t }` overrides any of them: they're
  registers, and a robot's motors may be set otherwise (OpenArm's DM4340s
  pack velocity into ±10 rad/s, not 8).
- `id`, `master_id`: where commands go, where answers come from.
- `peak_torque`: the most the motor gives, N m. TMAX by default, which is
  only the range torque is packed into: a DM-J4310 peaks at 7 N m, not 10.
- `rated_torque`: what the motor may give for as long as it likes, N m:
  what sets how hot it gets (see Heat). A third of `peak_torque` by default.
- `timeout_ms`: without a frame for this long, the motor disables itself with
  status `D` (communication lost). Off by default.
- `encoder: { delay_ms, bias, drift, noise, quantum }`: what the motor's
  encoder gets wrong, in radians (perfect by default). Its firmware closes
  its loop on that reading, and answers with it. Seeded: a run repeats.
- `transmission: { ratio, offset }`: when the motor and its joint don't turn
  together. The joint is at `offset + ratio θ` when the motor is at `θ`, and
  a motor torque `τ` puts `τ / ratio` on the joint. A gripper whose rotary
  motor slides its fingers has a ratio in metres per radian.

## Scenarios

The robot file says what the robot is; a scenario says what happens to it.
Motors break at set times, to check that what drives them handles it, or at
random, as real ones do:

```yaml
seed: 7                     # optional: the same seed, the same run
faults:
  - joint: openarm_joint4   # exactly then
    fault: dead
    at: 6 s
  - joint: any              # at random, about once a minute
    fault: freeze
    every: 60 s
    lasts: 0.5 s            # then it recovers; without it, it doesn't
```

```sh
drydock examples/openarm/robot.yml --scenario examples/openarm/elbow-dies.scenario.yml
```

- `fault`: `dead` (no torque), `weak` (a fifth of it), `freeze` (the
  encoder repeats its last reading), `noisy` (its noise twentyfold),
  `overheat` (it has lost most of its cooling: it heats five times as fast),
  `disconnected` (a loose connector: it neither hears frames nor answers,
  and goes on with its last command until its timeout, if it has one),
  `lossy` (about one frame in ten lost, each way). Faults add up: a dead
  motor that's also disconnected is still dead. A broken motor still says
  it's enabled until its firmware has something to say (an overheated one
  does): finding out is the bus master's job.
- `at` or `every`: when. `every` is the mean time between two, at random
  (any moment as likely as the next). In simulated time: `s` or `ms`.
- `joint`: one of the robot's, or `any`, drawn each time.
- `lasts`: how long until it recovers. Without it, it doesn't.
- `seed`: random faults are drawn from it, each fault from its own draws, so
  a run repeats, and adding a fault leaves the others' times as they were.

What drydock broke, and when, is the answer key for whatever runs against
it. It says so as it happens:

```
drydock: 6.000 s: openarm_joint4 breaks: dead
```

and with `--trace`, as a line of the trace, on the clock of the frames
around it:

```
{"joint":"openarm_joint4","fault":"dead","what":"breaks","sim_s":6,"at_ns":180650934766604}
```

## What's simulated

Every line below is shown by a test in [`tests/damiao.rs`](tests/damiao.rs),
one per feature, and so is every gap: CI turns them into a table on each
[run's summary](https://github.com/montmejat/drydock/actions/workflows/ci.yml).

| | |
|---|---|
| MIT command, to `id` | `kp (p_des - p) + kd (v_des - v) + t_ff`, every physics step, within `peak_torque` |
| Enable `FC`, disable `FD`, zero `FE`, clear error `FB` | each answered with feedback; enabling starts from no command; a motor in error stays in it until cleared |
| Feedback, on `master_id` | after every frame, as the real motors answer: status, id, position, velocity, torque, and the drive's and coil's temperatures |
| Over-temperature | past `OT_Value` (120 °C), status `B` (the drive) or `C` (the coil) and no torque, until cleared; see Heat |
| Registers, through `0x7FF` | read and write `OT_Value`, `MST_ID`, `ESC_ID`, `CTRL_MODE`, `PMAX`, `VMAX`, `TMAX`; refresh (`CC`) answers with feedback; save (`AA`) answers nothing |
| Packing | 16 bits of position, 12 of velocity, torque and gains, clamped to their ranges, as the real motors pack them |

Not yet, or not at all:

- **Other modes.** Position-velocity (`id + 0x100`), velocity (`id + 0x200`)
  and position-force (`id + 0x300`) frames are ignored, with a note. Writing another mode to
  `CTRL_MODE` is refused: the answer says it is still MIT mode (1).
- **Ids can't be rewritten** over the bus; they're the robot file's.
- **Registers other than the seven above** get no answer, with a note. That
  includes `TIMEOUT` (9): its units aren't in the sources below, so the
  timeout is set in the robot file instead.
- **No power supply.** Overvoltage, undervoltage, overcurrent and overload
  are never reported.
- **A lost frame is just lost.** On a real bus a corrupted frame is caught
  by its CRC, flagged with an error frame and sent again, and a node that
  errs too often goes bus-off. drydock's link faults lose frames outright:
  no error frames, retransmissions, error counters or bus-off.

## Heat

Each motor's coil and drive heat as it gives torque, and cool towards the
air (25 °C). Heat goes as the current squared, and the current as the
torque, so each heads for `25 °C + rise × (torque / rated torque)²`, as fast
as its time constant:

| | rise at rated torque | time constant |
|---|---|---|
| coil | 85 °C (110 °C, held forever) | 120 s |
| drive | 45 °C (70 °C) | 20 s |

Damiao's datasheets give a motor's rated and peak torque, not how fast or
how far it heats, so these are **assumptions**, chosen to agree with what
they do give: rated torque, held forever, stays under the threshold; peak
torque (three times rated) trips the drive within about five seconds. The
threshold is the motor's `OT_Value` register, 80 to 200 °C (openarm_can's
register table); what Damiao's motors leave the factory with isn't
published, so drydock's start at 120 °C. Past it, while enabled, a motor
stops with status `B` (drive) or `C` (coil), gives no torque, and says so
until the error is cleared; cleared, it runs again until it's past again.

A motor's rated torque defaults to a third of its peak. Where resellers
publish ratings, that's within a tenth of them: about 3 N m for a DM-J4310,
9 for a DM-J4340, 20 for a DM-J8009. `rated_torque` in the robot file sets
it.

## Time

**In real time** (the default), a physics step runs each timestep of the
wall clock (the scene's `<option timestep>`), and frames take as long as
they would on the real bus: each one's bits, stuffing included, at the bus's
bit rates, one frame at a time, the lowest id first when several wait (so a
master's commands go before the motors' answers). A frame reaches its motor
when it's across, and the answer, with the state of that moment, crosses in
turn. A bus kept busy says so, once a second:

```
drydock: vcan0: busy 96% of the last 1.0 s; frames waited up to 0.85 ms for it
```

(OpenArm on classic CAN at 500 Hz: it just fits.) If the simulation falls
more than 100 ms behind, it says so and carries on from the present.

`--trace <file>` writes every frame that crosses a bus, one JSON object a
line: its bus, id, sender, data, and when it was ready, started across and
arrived, in nanoseconds on `CLOCK_MONOTONIC`, the clock keel's traces use, so
a frame lines up with the message that carried it:

```
{"bus":"vcan0","id":4,"from":"master","fd":true,"data":"90157ff1eb6667ff","ready_ns":180648986126380,"start_ns":180648986283980,"end_ns":180648986335180}
```

**In lockstep** (`--lockstep <hz>`), time follows the bus master. Each frame
is answered at once, and once every motor has had a frame, `1/hz` seconds of
physics run. A master that sends each motor a command per cycle reads, in
the answers, what its previous commands did, a cycle later, as on a real bus.
Unlike a real bus, every run is the same, frame for frame, and runs as fast
as the master goes; frames take no time. Time stands still while the master is silent, so
timeouts never fire in lockstep unless a motor is left out of cycles that
the others complete.

## Layout

| | |
|---|---|
| `src/can/` | frames and SocketCAN; `timing.rs`, how long frames take and in what order |
| `src/run.rs` | the clock: real time or lockstep, over every bus |
| `src/damiao/` | `protocol.rs`, the frames; `motor.rs`, one motor's firmware; `bus.rs`, the motors on one bus |
| `src/drivetrain.rs` | what limits a motor's torque, how it moves its joint, what it senses |
| `src/effects.rs` | an imperfect encoder, and how a motor breaks |
| `src/scenario.rs` | when motors break: at set times, or at random |
| `src/world.rs`, `shim/world.c` | MuJoCo, through a small C shim, so Rust mirrors none of its structs |
| `src/dock.rs` | the robot: buses of motors on the world's joints |
| `src/robot.rs` | the robot file |
| `examples/arm` | a two-joint arm; `examples/wave.rs`, a bus master for it |
| `examples/openarm` | OpenArm v1; `hold.cpp`, a bus master on `openarm_can`; scenarios (`*.scenario.yml`) |

## Sources

The protocol is from Damiao's manual for the DM-J4310-2EC and their
reference driver, `DM_CAN.py` in DM_Control_Python: frame layouts, packing,
the special frames, registers and the factory ranges. OpenArm's
`openarm_can` agrees on every frame, and adds the DM3507's ranges.
