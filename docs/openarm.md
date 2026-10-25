# OpenArm

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

CI runs it on every pull request: OpenArm's library must find all eight
motors and hold the home pose.
