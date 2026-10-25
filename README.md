# drydock

A robot's motors on a virtual CAN bus, their physics simulated in MuJoCo.

## What it supports

drydock speaks one protocol, and tries to speak it completely: **Damiao's
motors** (DM-J4310, DM-J4340, DM-J8009 and the rest), in MIT mode and their
three other modes, as on OpenArm and many other arms and legs.

Every feature is shown by a test in [`tests/damiao.rs`](tests/damiao.rs),
and so is every gap: CI turns them into a table. The table, and what's not
there yet, is in [docs/simulation.md](docs/simulation.md).

## How it connects

Software that drives real motors over SocketCAN drives drydock's the same
way: open `vcan0` instead of `can0`, and the frames it sends move a simulated
robot, which answers as the motors would. Nothing in the robot's software
knows it is simulated, so the driver and its timing are tested too, not
replaced.

```
your software ──► SocketCAN (vcan0) ──► drydock ───────────────► MuJoCo
openarm_can,      the frames it would   each motor's firmware,   the robot's
keel, …           send to the motors    answering as the motor   physics
```

## Run it

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

Then the example arm:

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

## Docs

- [The robot file](docs/robot-file.md): a robot's scene, its buses and
  their motors.
- [Scenarios](docs/scenarios.md): motors that break, and the supply, at set
  times or at random.
- [What's simulated](docs/simulation.md): the support table and its gaps;
  heat, power and time; the sources.
- [OpenArm](docs/openarm.md): OpenArm v1, driven by keel or by OpenArm's own
  library.
- [Design](docs/architecture.md): the pieces, how a frame and a step go
  through them, the decisions behind them and what's left to do.
