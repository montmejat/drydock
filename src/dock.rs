//! A simulated robot: the world, its buses of motors, each motor on its
//! joint through its drivetrain, and the supply they run on. Frames that
//! reached the motors in, answers out, and steps of physics. When frames reach the motors, and how time
//! passes, is up to the caller (`run.rs`).

use crate::can::Frame;
use crate::damiao::bus::Motors;
use crate::damiao::motor::{Motor, Registers};
use crate::damiao::{bit_rate, status};
use crate::drivetrain::{Actuator, Drivetrain};
use crate::effects::{Breakdown, Encoder, Rng};
use crate::power::{self, Supply};
use crate::robot::Robot;
use crate::scenario::{Change, Fault, Happening, Scenario};
use crate::thermal::Thermal;
use crate::world::World;

pub struct Drydock {
    world: World,
    buses: Vec<Bus>,
    scenario: Option<Scenario>,
    /// What the scenario did, not yet asked for.
    happened: Vec<Happening>,
    /// Which frames a lossy link loses.
    links: Rng,
    supply: Supply,
    /// What it's set to, without faults.
    volts: f64,
    /// What a scenario set it to, each until when (or for good); the latest
    /// wins.
    supply_faults: Vec<(f64, Option<f64>)>,
}

/// One bus's motors, and their ways to their joints.
struct Bus {
    motors: Motors,
    drivetrains: Vec<Drivetrain>,
    /// Which motors have had a frame since the last cycle ended.
    heard: Vec<bool>,
}

impl Drydock {
    pub fn load(robot: &Robot) -> Result<Self, String> {
        let mut world = World::load(&robot.scene_path()?)?;
        if let Some(k) = &robot.keyframe {
            world.keyframe(k)?;
        }
        let mut buses = Vec::new();
        for (b, spec) in robot.buses.iter().enumerate() {
            let (mut motors, mut drivetrains) = (Vec::new(), Vec::new());
            let rate = bit_rate::code(spec.fd, spec.bitrate, spec.data_bitrate).expect("checked with the robot file");
            for (i, m) in spec.motors.iter().enumerate() {
                let joint = world.joint(&m.joint)?;
                if world.release_actuators(joint) > 0 {
                    eprintln!("drydock: the scene's actuator(s) on `{}` are off: its motor drives it", m.joint);
                }
                let mut regs = Registers::new(m.id, m.master_id, m.limits(), m.model.volts());
                (regs.timeout, regs.bit_rate) = (m.timeout(), rate);
                motors.push(Motor::new(m.joint.clone(), regs));
                let actuator = Actuator { peak_torque: m.peak_torque() };
                // Each motor's own noise, the same every run.
                let seed = robot.seed ^ (((b as u64) << 32) | (i as u64 + 1));
                let encoder = Encoder::new(m.encoder, seed);
                drivetrains.push(Drivetrain {
                    joint,
                    actuator,
                    transmission: m.transmission,
                    encoder,
                    thermal: Thermal::new(m.rated_torque()),
                    faults: Vec::new(),
                });
            }
            let heard = vec![false; motors.len()];
            buses.push(Bus { motors: Motors::new(motors, spec.fd, rate), drivetrains, heard });
        }
        let links = Rng::new(robot.seed ^ 0x6c696e6b);
        let supply = match robot.power {
            Some(spec) => Supply::bench(spec, robot.motors().count()),
            None => Supply::ideal(),
        };
        let volts = supply.set;
        Ok(Self { world, buses, scenario: None, happened: Vec::new(), links, supply, volts, supply_faults: Vec::new() })
    }

    /// What happens to the robot as time passes.
    pub fn set_scenario(&mut self, scenario: Scenario) {
        self.scenario = Some(scenario);
    }

    /// What broke and recovered since this was last asked, in order.
    pub fn happenings(&mut self) -> Vec<Happening> {
        std::mem::take(&mut self.happened)
    }

    /// The motors' joints, counted across buses as a scenario counts them.
    pub fn joint_name(&self, motor: usize) -> &str {
        let (b, i) = self.locate(motor);
        &self.buses[b].motors.motors()[i].joint
    }

    fn locate(&self, mut motor: usize) -> (usize, usize) {
        for (b, bus) in self.buses.iter().enumerate() {
            if motor < bus.drivetrains.len() {
                return (b, motor);
            }
            motor -= bus.drivetrains.len();
        }
        panic!("no motor {motor}")
    }

    /// The scenario's faults that start or end by `now`.
    fn faults(&mut self, now: f64) {
        let count = self.buses.iter().map(|b| b.drivetrains.len()).sum();
        let mut motor = 0;
        for b in &mut self.buses {
            for d in &mut b.drivetrains {
                for breakdown in d.recover(now) {
                    let fault = Fault::Motor { motor, breakdown };
                    self.happened.push(Happening { at: now, fault, what: Change::Recovers });
                }
                motor += 1;
            }
        }
        let (over, on): (Vec<_>, Vec<_>) =
            self.supply_faults.drain(..).partition(|&(_, until)| until.is_some_and(|u| u <= now));
        self.supply_faults = on;
        for (volts, _) in over {
            self.happened.push(Happening { at: now, fault: Fault::Supply { volts }, what: Change::Recovers });
        }
        let due = self.scenario.as_mut().map(|s| s.due(now, count)).unwrap_or_default();
        for h in due {
            if let Change::Breaks { until } = h.what {
                match h.fault {
                    Fault::Motor { motor, breakdown } => {
                        let (b, i) = self.locate(motor);
                        self.buses[b].drivetrains[i].faults.push((breakdown, until));
                    }
                    Fault::Supply { volts } => self.supply_faults.push((volts, until)),
                }
            }
            self.happened.push(Happening { at: now, ..h });
        }
        self.supply.set = self.supply_faults.last().map_or(self.volts, |&(v, _)| v);
    }

    /// The bus's voltage at the motors, V.
    pub fn volts(&self) -> f64 {
        self.supply.volts()
    }

    /// Bus `bus`'s motors, in the robot file's order.
    pub fn motors(&self, bus: usize) -> &[Motor] {
        self.buses[bus].motors.motors()
    }

    /// The peak torque of bus `bus`'s motor `i`.
    pub fn peak_torque(&self, bus: usize, i: usize) -> f64 {
        self.buses[bus].drivetrains[i].actuator.peak_torque
    }

    /// Simulated time, seconds.
    pub fn time(&self) -> f64 {
        self.world.time()
    }

    pub fn timestep(&self) -> f64 {
        self.world.timestep()
    }

    /// A frame reached the motors on bus `bus`: the answer, if one answers.
    ///
    /// A motor whose link is down neither hears the frame nor answers; a
    /// lossy link loses about one frame in ten, each way. Either way the
    /// frame was on the bus, so a lockstep cycle counts it.
    pub fn deliver(&mut self, bus: usize, frame: &Frame) -> Option<Frame> {
        /// One frame in ten, on a lossy link.
        const LOSS: f64 = 0.1;
        let now = self.world.time();
        let (world, b, links) = (&self.world, &mut self.buses[bus], &mut self.links);
        if let Some(i) = b.motors.addressee(frame) {
            b.heard[i] = true;
            let link = &b.drivetrains[i];
            if link.has(Breakdown::Disconnected) || (link.has(Breakdown::Lossy) && links.unit() < LOSS) {
                return None;
            }
        }
        let drivetrains = &mut b.drivetrains;
        let state = |i: usize| {
            let d = &mut drivetrains[i];
            let (q, v) = world.state(d.joint);
            d.sense(now, q, v)
        };
        let delivery = b.motors.deliver(frame, now, state)?;
        b.heard[delivery.motor] = true;
        if b.drivetrains[delivery.motor].has(Breakdown::Lossy) && links.unit() < LOSS {
            return None;
        }
        delivery.answer
    }

    /// One step of physics, under the torques the motors give now, and
    /// what they draw from the supply for it.
    pub fn step(&mut self) {
        let now = self.world.time();
        let dt = self.world.timestep();
        self.faults(now);
        let volts = self.supply.volts();
        let mut watts = 0.0;
        for b in &mut self.buses {
            for (i, d) in b.drivetrains.iter_mut().enumerate() {
                let (q, v) = self.world.state(d.joint);
                let (theta, omega) = d.sense(now, q, v);
                let motor = b.motors.motor_mut(i);
                motor.power(volts);
                let mut asked = motor.law(now, theta, omega);
                if asked != 0.0 && d.has(Breakdown::Shorted) {
                    motor.trip(status::OVERCURRENT);
                    asked = 0.0;
                }
                let torque = d.torque(asked);
                motor.applied(torque);
                d.heat(dt, torque);
                motor.temperatures(d.thermal.drive, d.thermal.coil);
                if motor.on() {
                    let (_, speed) = d.transmission.to_motor(q, v);
                    watts += power::draw(torque, speed, d.thermal.rated_torque);
                }
                self.world.set_force(d.joint, d.transmission.to_joint(torque));
            }
        }
        self.supply.step(dt, watts);
        self.world.step();
    }

    /// Whether every motor, on every bus, has had a frame since this last
    /// said yes: a cycle of the bus masters, in lockstep.
    pub fn cycle_done(&mut self) -> bool {
        if self.buses.iter().all(|b| b.heard.iter().all(|&h| h)) {
            for b in &mut self.buses {
                b.heard.fill(false);
            }
            return true;
        }
        false
    }
}
