# Scenarios

The [robot file](robot-file.md) says what the robot is; a scenario says what happens to it.
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
  - supply: off             # the motors' supply: off, or a voltage (`18 V`)
    at: 20 s
    lasts: 1 s
```

```sh
drydock examples/openarm/robot.yml --scenario examples/openarm/elbow-dies.scenario.yml
```

- `fault`: `dead` (no torque), `weak` (a fifth of it), `freeze` (the
  encoder repeats its last reading), `noisy` (its noise twentyfold),
  `overheat` (it has lost most of its cooling: it heats five times as fast),
  `disconnected` (a loose connector: it neither hears frames nor answers,
  and goes on with its last command until its timeout, if it has one),
  `lossy` (about one frame in ten lost, each way), `shorted` (a phase
  shorted: as soon as it drives, it stops with over-current). Faults add up: a dead
  motor that's also disconnected is still dead. A broken motor still says
  it's enabled until its firmware has something to say (an overheated one
  does): finding out is the bus master's job.
- `at` or `every`: when. `every` is the mean time between two, at random
  (any moment as likely as the next). In simulated time: `s` or `ms`.
- `joint`: one of the robot's, or `any`, drawn each time.
- `supply`: instead of a joint and its fault, the supply goes `off`, or to
  a voltage: a brownout (`12 V`), or a supply set too high (`36 V`). The
  latest one in force wins.
- `lasts`: how long until it recovers. Without it, it doesn't.
- `seed`: random faults are drawn from it, each fault from its own draws, so
  a run repeats, and adding a fault leaves the others' times as they were.

What drydock broke, and when, is the answer key for whatever runs against
it. It says so as it happens:

```
drydock: 6.000 s: openarm_joint4 breaks: dead
drydock: 15.000 s: the supply goes off, for 0.500 s
```

and with `--trace`, as a line of the trace, on the clock of the frames
around it:

```
{"joint":"openarm_joint4","fault":"dead","what":"breaks","sim_s":6,"at_ns":180650934766604}
{"supply_v":0,"what":"breaks","for_s":0.5,"sim_s":15,"at_ns":180659934766604}
```
