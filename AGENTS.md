# Working on drydock

Instructions for any agent changing this repository (Claude Code reads them
through `CLAUDE.md`). The README is what drydock is; `docs/architecture.md`
is how it works and why.

## What drydock promises

- **To be the hardware, frame for frame.** Software that drives Damiao's
  motors over SocketCAN must not be able to tell drydock from the real
  motors: the same frames, the same answers, the same timing on the bus.
  When the real motors and a simpler model disagree, the real motors win.
  Every claim about a protocol comes from a source (README, Sources), never
  from a guess; where the sources are silent, say so rather than invent.
- **One protocol, done completely**: Damiao's MIT mode. Its coverage is
  `tests/damiao.rs`: one test per feature, and an ignored test with its
  reason for each gap. A new feature, or a gap closed, changes that file;
  CI turns it into the support table.
- **No dependency on keel**, or on any one bus master. drydock is useful to
  anyone with SocketCAN software.
- **MuJoCo only through `shim/world.c`.** Rust never mirrors MuJoCo's
  structs. MuJoCo is one documented download; the build never reaches the
  network.

## What must not break

The robot file's format (`src/robot.rs`), the frames on the bus, and the
command line. Changing any of them is a breaking change: say so in the pull
request (label `breaking`), and update the README and the examples with it.

## Before saying a change is done

```sh
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test
```

Then, when the change touches the bus, the clock or a motor, run it against
a bus master on `vcan0` (the user creates it; it needs sudo):

```sh
sudo ip link add dev vcan0 type vcan && sudo ip link set vcan0 mtu 72 up
cargo run --release -- examples/arm/robot.yml &
cargo run --release --example wave -- examples/arm/robot.yml --seconds 3
```

CI runs all of this on every pull request. A change is done when CI passes,
the README says what changed for someone using drydock, and the examples
still run.

## How the code is written

- **Designed for what it supports today.** No flags, abstractions or
  fallbacks for features that might come; refactor when one does. Remove
  what isn't used.
- **Plain, short sentences** in comments, docs and messages, written for
  someone who doesn't know the code. Comments say why, not what. Facts with
  numbers ("frames waited up to 7.2 ms"), not verdicts.
- **Errors say what's wrong and how to fix it**, with the exact command when
  there is one. The command line guides; no setup scripts.
- **Tests show behaviour over the bus** where they can (`tests/`), as a bus
  master would see it, rather than reaching into internals.
- Format with `rustfmt.toml` (120 columns); format only the files you touch.

## Working with the user

- Work on a branch and open a pull request; the user reviews the diff there
  and comments. Its title becomes the release notes' line: write it for
  someone upgrading. Label it `breaking`, `feature` or `fix`.
- Commit and push only when asked. Never force-push or rewrite `master`.
- Ask before anything that's hard to undo or visible to others (releases,
  deleting branches, changing repository settings).
