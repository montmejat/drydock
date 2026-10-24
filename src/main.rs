use std::process::ExitCode;

use drydock::can::Socket;
use drydock::dock::Drydock;
use drydock::robot::Robot;
use drydock::run::{self, Line};

const USAGE: &str = "\
drydock: a robot's motors on virtual CAN buses, simulated in MuJoCo

usage: drydock <robot.yml> [--lockstep <hz>]

  --lockstep <hz>    step 1/hz seconds of physics each time every motor has
                     had a frame, instead of following the wall clock";

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
    let (mut path, mut lockstep) = (None, None);
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
            _ if arg.starts_with('-') => return Err(format!("no option {arg}\n\n{USAGE}")),
            _ if path.is_none() => path = Some(arg),
            _ => return Err(format!("one robot file at a time\n\n{USAGE}")),
        }
    }
    let path = path.ok_or(USAGE)?;
    let robot = Robot::load(&path)?;
    let dock = Drydock::load(&robot)?;
    if let Some(hz) = lockstep {
        run::steps_per_period(dock.timestep(), 1.0 / hz)?;
    }
    let mut lines = Vec::new();
    for bus in &robot.buses {
        let name = &bus.can;
        let socket = Socket::open(name).map_err(|e| {
            format!("can't open the CAN interface `{name}`: {e}\n(`sudo scripts/vcan.sh {name}` makes a virtual one)")
        })?;
        if bus.fd && !socket.fd_capable() {
            return Err(format!(
                "the motors on `{name}` are on CAN FD, but it carries classic frames only\n\
                 (`sudo scripts/vcan.sh {name}` makes it carry both)"
            ));
        }
        lines.push(Line { name: name.clone(), socket, rates: bus.rates(), reply: bus.reply_us / 1e6 });
    }

    let pace = match lockstep {
        Some(hz) => format!("in lockstep at {hz} Hz"),
        None => "in real time".into(),
    };
    eprintln!("drydock: {}, {pace}, {} ms steps", robot.scene.display(), dock.timestep() * 1e3);
    for (b, bus) in robot.buses.iter().enumerate() {
        let kind = match bus.fd {
            true => format!("CAN FD, {} / {} Mbit/s", bus.bitrate / 1e6, bus.data_bitrate / 1e6),
            false => format!("classic CAN, {} Mbit/s", bus.bitrate / 1e6),
        };
        eprintln!("  {} ({kind})", bus.can);
        for (i, m) in dock.motors(b).iter().enumerate() {
            eprintln!("    {:<22} {:#05x} -> {:#05x}  peak {} N m", m.joint, m.id, m.master_id, dock.peak_torque(b, i));
        }
    }
    let result = match lockstep {
        Some(hz) => run::lockstep(dock, &lines, 1.0 / hz),
        None => run::realtime(dock, &lines),
    };
    result.map_err(|e| format!("a bus: {e}"))
}
