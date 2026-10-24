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

## Try it

```sh
scripts/setup.sh                 # fetches MuJoCo 3.16.0 into ~/.cache/drydock
sudo scripts/vcan.sh             # makes vcan0, once per boot
cargo build --release --all-targets

./target/release/drydock examples/arm/robot.yml
```

In another terminal, a bus master swings the example arm's two joints:

```sh
./target/release/examples/wave examples/arm/robot.yml
```

```
t =   1.0 s  shoulder: +0.538 rad -1.57 N m [1]  elbow: +0.010 rad -0.32 N m [1]
t =   1.5 s  shoulder: +0.396 rad -1.69 N m [1]  elbow: -0.494 rad -0.26 N m [1]
```

`cargo test` runs the protocol's and the motor's unit tests, and the arm in
MuJoCo frame by frame without a bus.

## A robot

A robot is two files. The **scene** (MJCF) says how it moves: links, masses,
joints, rotor inertia (`armature`), damping, friction. The **robot file**
says what drives it:

```yaml
scene: arm.xml          # relative to this file
keyframe: home          # optional: where it starts
bus: vcan0              # optional, vcan0 by default
motors:
  - { joint: shoulder, model: DM4340, id: 0x01, master_id: 0x11 }
  - { joint: elbow, model: DM4310, id: 0x02, master_id: 0x12, peak_torque: 7, timeout_ms: 100 }
```

- `joint`: a hinge or slide joint in the scene. If the scene has its own
  actuators on it (MuJoCo Menagerie's models do), they're silenced: the
  motor drives the joint now.
- `model`: sets PMAX, VMAX, TMAX to the factory's, as Damiao's reference
  driver lists them.
- `id`, `master_id`: where commands go, where answers come from.
- `peak_torque`: the most the motor gives, N m. TMAX by default, which is
  only the range torque is packed into: a DM-J4310 peaks at 7 N m, not 10.
- `timeout_ms`: without a frame for this long, the motor disables itself with
  status `D` (communication lost). Off by default.

## What's simulated

| | |
|---|---|
| MIT command, to `id` | `kp (p_des - p) + kd (v_des - v) + t_ff`, every physics step, within `peak_torque` |
| Enable `FC`, disable `FD`, zero `FE`, clear error `FB` | each answered with feedback; enabling starts from no command; a motor in error stays in it until cleared |
| Feedback, on `master_id` | after every frame, as the real motors answer: status, id, position, velocity, torque, temperatures |
| Registers, through `0x7FF` | read and write `MST_ID`, `ESC_ID`, `CTRL_MODE`, `PMAX`, `VMAX`, `TMAX`; refresh (`CC`) answers with feedback; save (`AA`) answers nothing |
| Packing | 16 bits of position, 12 of velocity, torque and gains, clamped to their ranges, as the real motors pack them |

Not yet, or not at all:

- **Other modes.** Position-velocity (`id + 0x100`) and velocity
  (`id + 0x200`) frames are ignored, with a note. Writing another mode to
  `CTRL_MODE` is refused: the answer says it is still MIT mode (1).
- **Ids can't be rewritten** over the bus; they're the robot file's.
- **Registers other than the six above** get no answer, with a note. That
  includes `TIMEOUT` (9): its units aren't in the sources below, so the timeout is set in the robot file instead.
- **Temperatures** are always 25 °C. Nothing overheats, and no fault but
  communication loss happens on its own.
- **CAN FD.** Damiao's motors speak it too; drydock speaks classic CAN only.

## Time

**In real time** (the default), a physics step runs each timestep of the
wall clock (the scene's `<option timestep>`), and frames are answered as they
come, with the state of the moment. If the simulation falls more than
100 ms behind, it says so and carries on from the present.

**In lockstep** (`--lockstep <hz>`), time follows the bus master. Each frame
is answered at once, and once every motor has had a frame, `1/hz` seconds of
physics run. A master that sends each motor a command per cycle reads, in
the answers, what its previous commands did, a cycle later, as on a real bus.
Unlike a real bus, every run is the same, frame for frame, and runs as fast
as the master goes. Time stands still while the master is silent, so
timeouts never fire in lockstep unless a motor is left out of cycles that
the others complete.

## Layout

| | |
|---|---|
| `src/damiao.rs` | the protocol: frames, packing, the motors' factory ranges |
| `src/motor.rs` | one motor: its state, its answers, its torque |
| `src/world.rs`, `shim/world.c` | MuJoCo, through a small C shim, so Rust mirrors none of its structs |
| `src/dock.rs` | a robot: frames in, frames out, steps of physics |
| `src/run.rs` | real time and lockstep |
| `src/can.rs` | SocketCAN |
| `examples/arm` | a two-joint arm; `examples/wave.rs`, a bus master for it |

## Sources

The protocol is from Damiao's manual for the DM-J4310-2EC and their
reference driver, `DM_CAN.py` in DM_Control_Python: frame layouts, packing,
the special frames, registers and the factory ranges.
