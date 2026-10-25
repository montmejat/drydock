# What's simulated

Two protocols, each shown feature by feature by a test, one per feature, and
so is every gap: CI turns them into tables on each
[run's summary](https://github.com/montmejat/drydock/actions/workflows/ci.yml).
Damiao's motors on CAN ([`tests/damiao.rs`](../tests/damiao.rs)) first, then
Dynamixel servos on a serial line ([`tests/dynamixel.rs`](../tests/dynamixel.rs)).

## Damiao's motors

| | |
|---|---|
| MIT command, to `id` | `kp (p_des - p) + kd (v_des - v) + t_ff`, every physics step, within `peak_torque` |
| Position-velocity, to `id + 0x100` | to `p_des` no faster than `v_des` (two floats); the motor's own position and speed loops |
| Velocity, to `id + 0x200` | at `v_des` (one float, four bytes); its own speed loop |
| Position-force, to `id + 0x300` | to `p_des` no faster than `v_des`, pushing with at most a share `i_des` of its maximum (TMAX) |
| Modes, `CTRL_MODE` (1 to 4) | a motor takes commands in its mode only: others are ignored, unanswered, with a note; a new mode starts from no command |
| Enable `FC`, disable `FD`, zero `FE`, clear error `FB` | each answered with feedback; enabling starts from no command; a motor in error stays in it until cleared |
| Feedback, on `master_id` | after every frame, as the real motors answer: status, id, position, velocity, torque, and the drive's and coil's temperatures |
| Over-temperature | past `OT_Value` (120 °C), status `B` (the drive) or `C` (the coil) and no torque, until cleared; see [Heat](#heat) |
| Under- and overvoltage | enabled, below `UV_Value` (15 V) or above `OV_Value` (32 V, or 52 V for a 48 V motor), status `9` or `8`, until cleared; see [Power](#power) |
| Over-current | a `shorted` motor, as soon as it drives: status `A`, until cleared |
| Power off and on | below 10 V a motor neither hears nor answers; back, it starts disabled, from the registers it last saved |
| Registers, through `0x7FF` | every register in Damiao's table, read and written within its range; a refused write answers with the value kept; refresh (`CC`) answers with feedback; save (`AA`) answers done, while disabled |
| Ids, `ESC_ID` and `MST_ID` | a new id takes effect at once: commands go to it, answers come from the new master id |
| Timeout, `TIMEOUT` | in 50 µs; 0 for none |
| Bit rate, `CAN_BR` (0 to 9) | answered at the old rate; then a motor at another rate than the bus's neither hears nor answers, until drydock restarts |
| Output position, `xout` | as feedback reports it |
| Packing | 16 bits of position, 12 of velocity, torque and gains, clamped to their ranges, as the real motors pack them |

Not yet, or not at all:

- **The other modes' loop gains are assumed.** Damiao sets them with
  registers (`KP_ASR`, `KI_ASR`, `KP_APR`) whose defaults it doesn't
  publish; drydock's match the MIT gains OpenArm's driver uses (a speed gain
  of TMAX / 20 per rad/s, an integral at ten per second, a position gain of
  20 per second), and the registers aren't simulated.
- **Registers drydock has no model for.** Their values aren't published
  (versions, the motor's resistance, inductance and gear ratio, the factory
  loop gains), so they get no answer until written, with a note, rather
  than a made-up one. Written within their range, they keep the value and
  answer with it, but nothing changes: the other modes' gains among them.
- **Two motors on one id.** A write of another motor's `ESC_ID` is refused,
  with a note: on a real bus both motors would take its frames.
- **Overload** (status `E`) is never reported: Damiao's sources don't say
  what trips it. `OC_Value` is kept, but over-current comes only from a
  `shorted` fault: the sources don't say what current it's a share of.
- **A lost frame is just lost.** On a real bus a corrupted frame is caught
  by its CRC, flagged with an error frame and sent again, and a node that
  errs too often goes bus-off. drydock's link faults lose frames outright,
  and a motor at another bit rate is silent: no error frames,
  retransmissions, error counters or bus-off. vcan has no controller to
  report them.

## Dynamixel servos

Dynamixel's Protocol 2.0, on a half-duplex serial line, and the XL330-M288,
as on Pollen's Microduck. drydock makes the line a pseudo-terminal and links
it where the robot file says (`port`): the robot's software opens it as its
serial port, and can't tell it from the real one.

| | |
|---|---|
| Packets | header, id, length, instruction, byte stuffing and CRC-16, as the spec lays them out; a packet whose bytes stop for 1.5 ms is dropped, as a servo drops it |
| Ping, Read, Write, Reg Write and Action, Reboot, Factory Reset | each answered with a status packet after the servo's Return Delay Time, as its Status Return Level allows |
| Sync Read and Write, Bulk Read and Write | reads answered by each servo in turn, in the order asked |
| Fast Sync Read, Fast Bulk Read | one packet from every servo asked; one missing breaks it, as on a real bus |
| The control table | every item of the XL330's, at its address, size and default: writes whole items only, within their ranges, EEPROM only with torque off; goals within the position limits; errors as the spec numbers them |
| Position control (mode 3) | BAM's model of the XL330: the firmware's P loop gives a duty cycle, within the PWM and current limits, driving a DC motor with back-EMF; friction that grows with the load on the gearbox, as MuJoCo's own, so a held joint sticks |
| Present values | position (with the homing offset), velocity, current, PWM, input voltage, the realtime tick, moving |
| Input voltage | outside the voltage limits, with the input-voltage bit in Shutdown, torque goes off and the alert bit is set until a reboot |
| Baud rate | a servo set to another rate than the bus's neither hears nor answers |

**BAM**, Rhoban's "Better Actuator Models" (Apache-2.0), is a model of the
XL330 measured on real servos, and the one Pollen trains the Microduck's
policies against. drydock uses its `m6` parameters: a torque constant of
0.366 N m/A, 2.81 Ω, and its friction. Its P gain divisor (256, where
ROBOTIS says 128) is what BAM measured.

Not yet, or not at all:

- **Other operating modes**, the I and D gains, feedforward, profiles, the
  bus watchdog and indirect addresses are kept as written but change
  nothing.
- **No heat**: the temperature reads 25 °C, and overheating, overload and
  electrical shock are never reported.
- **An error outside Shutdown isn't flagged.** ROBOTIS doesn't say whether
  it is; Pollen's runtime sees no alert with the input-voltage bit clear.
- **Firmware v46**: what drydock's servos report, the oldest that answers
  Fast Sync Read. Real ones vary.
- **One sender at a time**: on a real half-duplex line, two senders at once
  garble each other; drydock queues them.

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

## Power

Without `power` in the [robot file](robot-file.md), the motors run from an ideal 24 V
supply: it never sags, and takes back what braking motors give, as a large
battery would. With it, they run from a bench supply set to `volts`,
through `ohms` of wiring (there and back, none by default):

- It **sags** as the motors draw: each draws what it gives its joint
  (torque × speed), what its coil loses as heat, and what its electronics
  need.
- It **can't take current back.** A motor braking gives back what it takes
  from its joint, and that charges the capacitors across the bus until
  something draws it. Swing the example arm fast on 24 V and brake, and the
  bus passes 32 V: both motors stop with overvoltage.

| | per motor | |
|---|---|---|
| coil losses | 1 W per N m of rated torque, at rated torque, going as the torque squared | assumed |
| electronics | 1 W | assumed |
| capacitance | 470 µF | assumed |

Damiao doesn't publish these, so they are **assumptions**, about what small
drives of this size have. What the motors do with the voltage is their
firmware's: below `UV_Value` or above `OV_Value`, while enabled, a motor
stops with status `9` or `8` and gives no torque until the error is cleared.
Damiao's manual recommends at least 15 V and at most 32 V (52 V for a 48 V
motor, by the model's name, and the DM-J10010L), and their factory values
aren't published, so drydock's motors start there. Below 10 V, the least
`UV_Value` takes, a motor is off (assumed): it neither hears nor answers,
and comes back disabled, from the registers it last saved (`AA`), its zero
kept.

A [scenario](scenarios.md) sets the supply (`supply: off`, `supply: 12 V`) as it breaks a
motor; `examples/openarm/power-cut.scenario.yml` browns it out, then cuts it.

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

On a serial line, bytes take ten bits each at its baud rate, one sender at
a time, and a servo's answer is ready its Return Delay Time after the packet
it answers is across (500 µs as it leaves the factory: the Microduck sets
0). With `--trace`, they go there too, a line per crossing:

```
{"serial":"/tmp/drydock-xl330","from":"master","data":"fffffd0001030001194e","ready_ns":…,"start_ns":…,"end_ns":…}
```

**In lockstep** (`--lockstep <hz>`), time follows the bus master. Each frame
or packet is answered at once, and once every motor has had one, `1/hz`
seconds of physics run. A master that sends each motor a command per cycle reads, in
the answers, what its previous commands did, a cycle later, as on a real bus.
Unlike a real bus, every run is the same, frame for frame, and runs as fast
as the master goes; frames take no time. Time stands still while the master is silent, so
timeouts never fire in lockstep unless a motor is left out of cycles that
the others complete.

## Sources

The protocol is from Damiao's manual for the DM-J4310-2EC and their
reference driver, `DM_CAN.py` in DM_Control_Python: frame layouts, packing,
the special frames, registers and the factory ranges. OpenArm's
`openarm_can` agrees on every frame, and adds the DM3507's ranges.
Enactic's copy of Damiao's manual for the DM-J4340P-2EC
([damiao.enactic.ai](https://damiao.enactic.ai/en/products/hardware/dm-j4340p-2ec-v1.0/))
gives the register table with each register's range, the bit-rate codes,
the save's answer, and the voltages and temperatures it recommends
protecting at. The `TIMEOUT` register's unit, 50 µs, is from the
[damiao-motor](https://damiao-motor.jia-xie.com/concept/registers/) Python
driver's documentation; Damiao's own doesn't say.

Dynamixel's packets are from ROBOTIS's
[Protocol 2.0](https://emanual.robotis.com/docs/en/dxl/protocol2/) e-Manual,
its worked examples the codec's tests, and the Fast Sync Read reply checked
against Pollen's [rustypot](https://github.com/pollen-robotics/rustypot) 1.8.0,
the library the Microduck's runtime uses. The XL330's control table is from
its [e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/); its
motor and friction from Rhoban's [BAM](https://github.com/Rhoban/bam)
(`bam/params/xl330/m6.json`). The Microduck's settings (Return Delay 0,
Shutdown 52) are from Pollen's
[runtime](https://github.com/pollen-robotics/microduck).
