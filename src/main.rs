use std::process::ExitCode;

use drydock::can::Socket;
use drydock::dock::Drydock;
use drydock::robot::Robot;
use drydock::run::{self, BusTrace, Line};
use drydock::scenario::Scenario;

const USAGE: &str = "\
drydock: a robot's motors on virtual CAN buses, simulated in MuJoCo

usage: drydock <robot.yml> [--scenario <file>] [--lockstep <hz>] [--trace <file>]

  --scenario <file>  what happens to the robot: motors breaking, the supply
                     failing, at set times or at random
                     (see examples/openarm/*.scenario.yml)
  --lockstep <hz>    step 1/hz seconds of physics each time every motor has
                     had a frame, instead of following the wall clock
  --trace <file>     write every frame that crosses a bus, with when it was
                     ready, started across and arrived, and every fault as it
                     starts and ends (real time)";

fn main() -> ExitCode {
    match main_() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("drydock: {e}");
            ExitCode::FAILURE
        }
    }
}

fn main_() -> Result<(), String> {
    let mut args = std::env::args().skip(1);
    let (mut path, mut lockstep, mut scenario, mut trace) = (None, None, None, None);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            "--lockstep" => {
                let hz = args.next().ok_or("--lockstep needs a rate")?;
                let hz: f64 = hz.parse().map_err(|_| format!("--lockstep {hz}: not a number"))?;
                if hz.is_nan() || hz <= 0.0 {
                    return Err("--lockstep needs a rate above zero".into());
                }
                lockstep = Some(hz);
            }
            "--scenario" => scenario = Some(args.next().ok_or("--scenario needs a file")?),
            "--trace" => trace = Some(args.next().ok_or("--trace needs a file")?),
            _ if arg.starts_with('-') => return Err(format!("no option {arg}\n\n{USAGE}")),
            _ if path.is_none() => path = Some(arg),
            _ => return Err(format!("one robot file at a time\n\n{USAGE}")),
        }
    }
    let path = path.ok_or(USAGE)?;
    if trace.is_some() && lockstep.is_some() {
        return Err("--trace records how long frames take on the bus, which only happens in real time".into());
    }
    let trace = trace.map(|t| BusTrace::create(t.as_ref()).map_err(|e| format!("can't write {t}: {e}"))).transpose()?;
    let robot = Robot::load(&path)?;
    let mut dock = Drydock::load(&robot)?;
    if let Some(file) = &scenario {
        let joints: Vec<String> = robot.motors().map(|(_, m)| m.joint.clone()).collect();
        dock.set_scenario(Scenario::load(file, &joints)?);
    }
    if let Some(hz) = lockstep {
        run::steps_per_period(dock.timestep(), 1.0 / hz)?;
    }
    let mut lines = Vec::new();
    for bus in &robot.buses {
        let name = &bus.can;
        let socket = Socket::open(name).map_err(|e| {
            format!(
                "can't open the CAN interface `{name}`: {e}. A virtual one, once per boot:\n\n  \
                 sudo ip link add dev {name} type vcan && sudo ip link set {name} mtu 72 up\n"
            )
        })?;
        if bus.fd && !socket.fd_capable() {
            return Err(format!(
                "the motors on `{name}` are on CAN FD, but it carries classic frames only. To carry both:\n\n  \
                 sudo ip link set {name} down && sudo ip link set {name} mtu 72 up\n"
            ));
        }
        lines.push(Line { name: name.clone(), socket, rates: bus.rates(), reply: bus.reply_us / 1e6 });
    }

    let pace = match lockstep {
        Some(hz) => format!("in lockstep at {hz} Hz"),
        None => "in real time".into(),
    };
    eprintln!("drydock: {}, {pace}, {} ms steps", robot.scene, dock.timestep() * 1e3);
    for (b, bus) in robot.buses.iter().enumerate() {
        let kind = match bus.fd {
            true => format!("CAN FD, {} / {} Mbit/s", bus.bitrate / 1e6, bus.data_bitrate / 1e6),
            false => format!("classic CAN, {} Mbit/s", bus.bitrate / 1e6),
        };
        eprintln!("  {} ({kind})", bus.can);
        for (i, m) in dock.motors(b).iter().enumerate() {
            eprintln!(
                "    {:<22} {:#05x} -> {:#05x}  peak {} N m",
                m.joint,
                m.id(),
                m.registers().master_id,
                dock.peak_torque(b, i)
            );
        }
    }
    if let Some(file) = &scenario {
        eprintln!("drydock: what happens to it: {file}");
    }
    let result = match lockstep {
        Some(hz) => run::lockstep(dock, &lines, 1.0 / hz),
        None => run::realtime(dock, &lines, trace),
    };
    result.map_err(|e| format!("a bus: {e}"))
}
