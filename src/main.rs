use std::process::ExitCode;

use drydock::can::Bus;
use drydock::dock::Drydock;
use drydock::robot::Robot;
use drydock::run;

const USAGE: &str = "\
drydock: a robot's motors on a virtual CAN bus, simulated in MuJoCo

usage: drydock <robot.yml> [--bus <interface>] [--lockstep <hz>]

  --bus <interface>  the CAN interface, instead of the robot file's (vcan0)
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
    let (mut path, mut bus, mut lockstep) = (None, None, None);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "-h" | "--help" => {
                println!("{USAGE}");
                return Ok(());
            }
            "--bus" => bus = Some(args.next().ok_or("--bus needs an interface")?),
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
    let interface = bus.unwrap_or_else(|| robot.bus.clone());
    let dock = Drydock::load(&robot)?;
    if let Some(hz) = lockstep {
        run::steps_per_period(dock.timestep(), 1.0 / hz)?;
    }
    let bus = Bus::open(&interface).map_err(|e| {
        format!(
            "can't open the CAN interface `{interface}`: {e}\n(`sudo scripts/vcan.sh {interface}` makes a virtual one)"
        )
    })?;

    if dock.can_fd() && !bus.fd_capable() {
        return Err(format!(
            "the motors are on CAN FD, but `{interface}` carries classic frames only\n\
             (`sudo scripts/vcan.sh {interface}` makes it carry both)"
        ));
    }
    let pace = match lockstep {
        Some(hz) => format!("in lockstep at {hz} Hz"),
        None => "in real time".into(),
    };
    let kind = if dock.can_fd() { "CAN FD" } else { "classic CAN" };
    eprintln!("drydock: {} on {interface} ({kind}), {pace}, {} ms steps", robot.scene.display(), dock.timestep() * 1e3);
    for m in dock.motors() {
        eprintln!("  {:<12} {:#05x} -> {:#05x}  peak {} N m", m.joint, m.id, m.master_id, m.peak_torque);
    }
    let result = match lockstep {
        Some(hz) => run::lockstep(dock, &bus, 1.0 / hz),
        None => run::realtime(dock, &bus),
    };
    result.map_err(|e| format!("the bus: {e}"))
}
