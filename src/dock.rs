//! A simulated robot: the world, its buses of motors (Damiao's on CAN,
//! Dynamixel servos on serial lines), each motor on its joint through its
//! drivetrain, and the supply they run on. Frames and packets that reached
//! the motors in, answers out, and steps of physics. When they reach the
//! motors, and how time passes, is up to the caller (`run.rs`).

use crate::can::Frame;
use crate::damiao::bus::Motors;
use crate::damiao::motor::{Motor, Registers};
use crate::damiao::{bit_rate, status};
use crate::drivetrain::{Actuator, Drivetrain};
use crate::dynamixel::bus::{Reply, Servos};
use crate::dynamixel::xl330::Xl330;
use crate::dynamixel::Packet;
use crate::effects::{Breakdown, Encoder, Rng};
use crate::power::{self, Supply};
use crate::robot::Robot;
use crate::scenario::{Change, Fault, Happening, Scenario};
use crate::thermal::Thermal;
use crate::world::World;

pub struct Drydock {
    world: World,
    buses: Vec<Bus>,
    serial: Vec<SerialBus>,
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

/// One serial bus's servos, and theirs.
struct SerialBus {
    servos: Servos,
    drivetrains: Vec<Drivetrain>,
    /// Which servos have had a packet since the last cycle ended.
    heard: Vec<bool>,
}

/// One frame in ten, or one packet, on a lossy link.
const LOSS: f64 = 0.1;

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
        let mut serial = Vec::new();
        for (b, spec) in robot.serial.iter().enumerate() {
            let (mut servos, mut drivetrains) = (Vec::new(), Vec::new());
            let baud = spec.baud_code().expect("checked with the robot file");
            for (i, m) in spec.motors.iter().enumerate() {
                let joint = world.joint(&m.joint)?;
                if world.release_actuators(joint) > 0 {
                    eprintln!("drydock: the scene's actuator(s) on `{}` are off: its servo drives it", m.joint);
                }
                servos.push(Xl330::new(m.joint.clone(), m.id, baud));
                let seed = robot.seed ^ ((((robot.buses.len() + b) as u64) << 32) | (i as u64 + 1));
                drivetrains.push(Drivetrain {
                    joint,
                    // What limits a servo's torque is its own firmware and motor (BAM).
                    actuator: Actuator { peak_torque: f64::INFINITY },
                    transmission: m.transmission,
                    encoder: Encoder::new(m.encoder, seed),
                    thermal: Thermal::new(f64::INFINITY),
                    faults: Vec::new(),
                });
            }
            let heard = vec![false; servos.len()];
            serial.push(SerialBus { servos: Servos::new(servos, baud), drivetrains, heard });
        }
        let links = Rng::new(robot.seed ^ 0x6c696e6b);
        let supply = match robot.power {
            Some(spec) => Supply::new(spec, robot.joints().len()),
            None => Supply::ideal(),
        };
        let volts = supply.set;
        Ok(Self {
            world,
            buses,
            serial,
            scenario: None,
            happened: Vec::new(),
            links,
            supply,
            volts,
            supply_faults: Vec::new(),
        })
    }

    /// What happens to the robot as time passes.
    pub fn set_scenario(&mut self, scenario: Scenario) {
        self.scenario = Some(scenario);
    }

    /// What broke and recovered since this was last asked, in order.
    pub fn happenings(&mut self) -> Vec<Happening> {
        std::mem::take(&mut self.happened)
    }

    /// The motors' joints, counted across buses as a scenario counts them:
    /// CAN buses first, then serial ones.
    pub fn joint_name(&self, motor: usize) -> &str {
        match self.locate(motor) {
            Place::Can(b, i) => &self.buses[b].motors.motors()[i].joint,
            Place::Serial(b, i) => &self.serial[b].servos.servos()[i].joint,
        }
    }

    fn locate(&self, mut motor: usize) -> Place {
        for (b, bus) in self.buses.iter().enumerate() {
            if motor < bus.drivetrains.len() {
                return Place::Can(b, motor);
            }
            motor -= bus.drivetrains.len();
        }
        for (b, bus) in self.serial.iter().enumerate() {
            if motor < bus.drivetrains.len() {
                return Place::Serial(b, motor);
            }
            motor -= bus.drivetrains.len();
        }
        panic!("no motor {motor}")
    }

    fn drivetrains_mut(&mut self) -> impl Iterator<Item = &mut Drivetrain> {
        let can = self.buses.iter_mut().flat_map(|b| b.drivetrains.iter_mut());
        can.chain(self.serial.iter_mut().flat_map(|b| b.drivetrains.iter_mut()))
    }

    /// The scenario's faults that start or end by `now`.
    fn faults(&mut self, now: f64) {
        let mut recovered = Vec::new();
        for (motor, d) in self.drivetrains_mut().enumerate() {
            for breakdown in d.recover(now) {
                recovered.push(Happening { at: now, fault: Fault::Motor { motor, breakdown }, what: Change::Recovers });
            }
        }
        let count = self.drivetrains_mut().count();
        self.happened.extend(recovered);
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
                        let d = match self.locate(motor) {
                            Place::Can(b, i) => &mut self.buses[b].drivetrains[i],
                            Place::Serial(b, i) => &mut self.serial[b].drivetrains[i],
                        };
                        d.faults.push((breakdown, until));
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

    /// An instruction packet reached the servos on serial bus `bus`: their
    /// answers, each with how long after the packet it's ready.
    ///
    /// As on CAN, a servo whose link is down neither hears nor answers, and
    /// a lossy link loses about one packet in ten, each way.
    pub fn deliver_packet(&mut self, bus: usize, packet: &Packet) -> Vec<Reply> {
        let (b, links) = (&mut self.serial[bus], &mut self.links);
        let drivetrains = &b.drivetrains;
        let heard = &mut b.heard;
        let mut link = |i: usize| {
            heard[i] = true;
            let d = &drivetrains[i];
            !(d.has(Breakdown::Disconnected) || (d.has(Breakdown::Lossy) && links.unit() < LOSS))
        };
        let mut answers = b.servos.deliver(packet, &mut link);
        answers.retain(|a| a.from.is_none_or(|i| !b.drivetrains[i].has(Breakdown::Lossy) || links.unit() >= LOSS));
        answers
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
        for b in &mut self.serial {
            for (i, d) in b.drivetrains.iter_mut().enumerate() {
                let (q, v) = self.world.state(d.joint);
                let (theta, omega) = d.sense(now, q, v);
                let ratio = d.transmission.ratio;
                // The load at the joint, at the horn: a joint torque τ is τ × ratio there.
                let load = self.world.load_torque(d.joint) * ratio;
                let out = b.servos.servo_mut(i).step(now, theta, omega, volts, load);
                let torque = d.torque(out.torque);
                self.world.set_force(d.joint, d.transmission.to_joint(torque));
                self.world.set_friction(d.joint, out.frictionloss / ratio.abs(), out.damping / (ratio * ratio));
                // What the H-bridge takes from the supply: the duty cycle's share of the motor's current.
                watts += volts * out.duty * out.current;
            }
        }
        self.supply.step(dt, watts);
        self.world.step();
    }

    /// Whether every motor, on every bus, has had a frame since this last
    /// said yes: a cycle of the bus masters, in lockstep.
    pub fn cycle_done(&mut self) -> bool {
        let all = |heard: &[bool]| heard.iter().all(|&h| h);
        if self.buses.iter().all(|b| all(&b.heard)) && self.serial.iter().all(|b| all(&b.heard)) {
            self.buses.iter_mut().for_each(|b| b.heard.fill(false));
            self.serial.iter_mut().for_each(|b| b.heard.fill(false));
            return true;
        }
        false
    }

    /// Serial bus `bus`'s servos, in the robot file's order.
    pub fn servos(&self, bus: usize) -> &[Xl330] {
        self.serial[bus].servos.servos()
    }
}

/// Where a motor is: its bus and its place on it.
enum Place {
    Can(usize, usize),
    Serial(usize, usize),
}
