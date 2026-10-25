# The robot file

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
power: { volts: 24, ohms: 0.1 }  # optional: a bench supply, through its wires (see Power, in simulation.md)
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
- `bitrate`, `data_bitrate`: how fast frames cross the bus (see [Time](simulation.md#time)).
  Damiao's motors run classic CAN at 125, 200, 250 or 500 kbit/s or
  1 Mbit/s, and CAN FD's data phase at 2, 2.5, 3.2, 4 or 5 Mbit/s: a bus at
  another rate is an error. Their `CAN_BR` register starts at the bus's.
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
  what sets how hot it gets (see [Heat](simulation.md#heat)). A third of `peak_torque` by default.
- `timeout_ms`: without a frame for this long, the motor disables itself with
  status `D` (communication lost). Off by default. It's the `TIMEOUT`
  register, in its units of 50 µs, which a bus master may write.
- `encoder: { delay_ms, bias, drift, noise, quantum }`: what the motor's
  encoder gets wrong, in radians (perfect by default). Its firmware closes
  its loop on that reading, and answers with it. Seeded: a run repeats.
- `transmission: { ratio, offset }`: when the motor and its joint don't turn
  together. The joint is at `offset + ratio θ` when the motor is at `θ`, and
  a motor torque `τ` puts `τ / ratio` on the joint. A gripper whose rotary
  motor slides its fingers has a ratio in metres per radian.
