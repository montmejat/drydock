# Microduck

Pollen's [Microduck](https://github.com/pollen-robotics/microduck): fifteen
Dynamixel XL330s and a body IMU on one serial bus at 1 Mbit/s, on a 2S
battery, as Pollen's runtime expects it. The model is Pollen's own, from
[microduck_rl](https://github.com/pollen-robotics/microduck_rl), which
drydock fetches on first run (the robot file names its repository and
commit). Its meshes are CC BY-NC-SA 4.0: fetched into drydock's cache,
never shipped with drydock.

```sh
drydock examples/microduck/robot.yml --lockstep 50
```

In lockstep, the duck stands from the first tick: time waits for the bus
master. In real time it starts as soon as drydock does, and an unpowered
duck falls over before anything drives it, as a real one would.

The bus is linked at `/tmp/microduck-tty`. On it, as on the robot:

- **The servos**, ids 20 to 24 (left leg), 30 to 34 (neck, head, mouth), 10
  to 14 (right leg), set up as Pollen's runtime sets them: no return delay,
  Shutdown 52, PWM slope 255, the knees counting from -512. A joint's angle
  is its servo's position in radians, centred at 2048, for every joint.
- **The mouth** (id 34) is in no model of the duck: its servo answers and
  holds where it's told.
- **The IMU board** (id 200), read with the servos each tick: the gyro and
  the orientation of the trunk, as an LSM6DSV16X mounted as on the zero3
  board would give them.

`examples/microduck/stand.rs` reads the duck as Pollen's runtime does each
tick, through Pollen's `rustypot` 1.8.0: one fast read of twelve bytes at
124, the IMU first. It finds every device, turns torque on where the joints
are, and checks the duck still stands, by the gravity the IMU gives:

```
ok   15 of 15 servos answer
ok   the IMU board answers
ok   the knees count from -512
ok   torque on where the joints are
ok   still standing: gravity [+0.116 +0.000 -0.993]
```

CI runs it on every pull request.

## Pollen's runtime

Pollen's runtime (`robotd`) talks to the duck through rustypot, as the check
does, so it should run against drydock with `--port /tmp/microduck-tty`.
That hasn't been tried yet. Two things to know first:

- **It powers the machine off on an empty battery.** With the battery's
  averaged voltage at 6.6 V or less, it runs `systemctl poweroff`. drydock's
  duck reads 7.4 V, but set `battery_empty_shutdown = false` in its
  parameters before pointing it at a simulation.
- **It warns of a frozen IMU** after 25 identical readings. drydock's gyro
  has a little noise (`noise` in the robot file), as a real one does.
