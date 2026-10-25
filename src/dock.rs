//! A simulated robot: the world, and its buses of motors, each motor on its
//! joint through its drivetrain. Frames that reached the motors in, answers
//! out, and steps of physics. When frames reach the motors, and how time
//! passes, is up to the caller (`run.rs`).

use crate::can::Frame;
use crate::damiao::bus::Motors;
use crate::damiao::motor::Motor;
use crate::drivetrain::{Actuator, Drivetrain};
use crate::effects::Encoder;
use crate::robot::Robot;
use crate::world::World;

pub struct Drydock {
    world: World,
    buses: Vec<Bus>,
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
            for (i, m) in spec.motors.iter().enumerate() {
                let joint = world.joint(&m.joint)?;
                if world.release_actuators(joint) > 0 {
                    eprintln!("drydock: the scene's actuator(s) on `{}` are off: its motor drives it", m.joint);
                }
                let timeout = m.timeout_ms.map(|ms| ms / 1e3);
                motors.push(Motor::new(m.joint.clone(), m.id, m.master_id, m.limits(), timeout));
                let actuator = Actuator { peak_torque: m.peak_torque() };
                // Each motor's own noise, the same every run.
                let seed = robot.seed ^ (((b as u64) << 32) | (i as u64 + 1));
                let encoder = Encoder::new(m.encoder, seed);
                drivetrains.push(Drivetrain { joint, actuator, transmission: m.transmission, encoder, fault: m.fault });
            }
            let heard = vec![false; motors.len()];
            buses.push(Bus { motors: Motors::new(motors, spec.fd), drivetrains, heard });
        }
        Ok(Self { world, buses })
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
    pub fn deliver(&mut self, bus: usize, frame: &Frame) -> Option<Frame> {
        let now = self.world.time();
        let (world, b) = (&self.world, &mut self.buses[bus]);
        let drivetrains = &mut b.drivetrains;
        let state = |i: usize| {
            let d = &mut drivetrains[i];
            let (q, v) = world.state(d.joint);
            d.sense(now, q, v)
        };
        let delivery = b.motors.deliver(frame, now, state)?;
        b.heard[delivery.motor] = true;
        delivery.answer
    }

    /// One step of physics, under the torques the motors give now.
    pub fn step(&mut self) {
        let now = self.world.time();
        for b in &mut self.buses {
            for (i, d) in b.drivetrains.iter_mut().enumerate() {
                let (q, v) = self.world.state(d.joint);
                let (theta, omega) = d.sense(now, q, v);
                let motor = b.motors.motor_mut(i);
                let torque = d.torque(now, motor.law(now, theta, omega));
                motor.applied(torque);
                self.world.set_force(d.joint, d.transmission.to_joint(torque));
            }
        }
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
