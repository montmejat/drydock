//! A simulated robot: the world, and the motors on its joints. Frames in,
//! frames out, and steps of physics; how time passes is up to the caller
//! (`run.rs`).

use std::collections::HashSet;

use crate::can::Frame;
use crate::damiao::{Register, RegisterReply, REGISTER_ID};
use crate::motor::Motor;
use crate::robot::{Robot, Transmission};
use crate::world::{Joint, World};

pub struct Drydock {
    world: World,
    motors: Vec<Motor>,
    joints: Vec<Joint>,
    transmissions: Vec<Transmission>,
    /// Whether the motors are on CAN FD: they answer in it, and don't
    /// understand it otherwise.
    can_fd: bool,
    /// Which motors have had a frame since the last cycle ended.
    heard: Vec<bool>,
    /// What has been said once, not to say it again.
    said: HashSet<String>,
}

impl Drydock {
    pub fn load(robot: &Robot) -> Result<Self, String> {
        let mut world = World::load(&robot.scene_path())?;
        if let Some(k) = &robot.keyframe {
            world.keyframe(k)?;
        }
        let mut motors = Vec::new();
        let mut joints = Vec::new();
        let mut transmissions = Vec::new();
        for spec in &robot.motors {
            let joint = world.joint(&spec.joint)?;
            let released = world.release_actuators(joint);
            if released > 0 {
                eprintln!("drydock: the scene's actuator(s) on `{}` are off: its motor drives it", spec.joint);
            }
            let limits = spec.limits();
            let peak = spec.peak_torque.unwrap_or(limits.t as f64);
            let timeout = spec.timeout_ms.map(|ms| ms / 1e3);
            motors.push(Motor::new(spec.joint.clone(), spec.id, spec.master_id, limits, peak, timeout));
            joints.push(joint);
            transmissions.push(spec.transmission);
        }
        let heard = vec![false; motors.len()];
        Ok(Self { world, motors, joints, transmissions, can_fd: robot.can_fd, heard, said: HashSet::new() })
    }

    pub fn motors(&self) -> &[Motor] {
        &self.motors
    }

    /// Simulated time, seconds.
    pub fn time(&self) -> f64 {
        self.world.time()
    }

    pub fn timestep(&self) -> f64 {
        self.world.timestep()
    }

    pub fn can_fd(&self) -> bool {
        self.can_fd
    }

    /// A frame from the bus, and the motor's answer if one answers, in CAN
    /// FD if the motors are on it.
    ///
    /// Motors on CAN FD understand classic frames too, as CAN FD
    /// controllers do; motors on classic CAN don't understand CAN FD (on a
    /// real bus, they would answer with error frames), so it's ignored, with
    /// a note.
    pub fn handle(&mut self, frame: &Frame) -> Option<Frame> {
        if frame.fd && !self.can_fd {
            self.say(format!(
                "a CAN FD frame to {:#05x}, but the motors are on classic CAN (can_fd: false): ignored",
                frame.id
            ));
            return None;
        }
        let can_fd = self.can_fd;
        self.answer(frame).map(|answer| answer.fd(can_fd))
    }

    fn answer(&mut self, frame: &Frame) -> Option<Frame> {
        let data = frame.eight()?;
        let now = self.world.time();
        if frame.id == REGISTER_ID {
            let request = Register::decode(&data)?;
            let i = self.motors.iter().position(|m| m.id == request.motor())?;
            self.heard[i] = true;
            let (q, v) = self.motor_state(i);
            let reply = self.motors[i].on_register(request, now, q, v);
            self.explain(i, request, reply.as_ref());
            return reply;
        }
        if let Some(i) = self.motors.iter().position(|m| m.id == frame.id) {
            self.heard[i] = true;
            let (q, v) = self.motor_state(i);
            return Some(self.motors[i].on_frame(&data, now, q, v));
        }
        // Position-velocity mode is at id + 0x100, velocity at + 0x200,
        // position-force at + 0x300.
        if let Some(m) = self.motors.iter().find(|m| [0x100, 0x200, 0x300].iter().any(|&o| frame.id == m.id + o)) {
            let joint = m.joint.clone();
            self.say(format!("{joint}: a frame for another mode than MIT, which is all drydock simulates"));
        }
        None
    }

    /// Says why a register request didn't do what it asked, once.
    fn explain(&mut self, i: usize, request: Register, reply: Option<&Frame>) {
        let joint = self.motors[i].joint.clone();
        match (request, reply) {
            (Register::Read { rid, .. } | Register::Write { rid, .. }, None) => {
                self.say(format!("{joint}: register {rid} isn't simulated: no answer"))
            }
            (Register::Write { rid, value, .. }, Some(reply))
                if reply.eight().and_then(|d| RegisterReply::decode(&d)).is_some_and(|r| r.value != value) =>
            {
                self.say(format!("{joint}: register {rid} can't take that value: it keeps its own"))
            }
            _ => {}
        }
    }

    fn say(&mut self, message: String) {
        if self.said.insert(message.clone()) {
            eprintln!("drydock: {message}");
        }
    }

    /// Motor `i`'s angle and velocity, through its transmission.
    fn motor_state(&self, i: usize) -> (f64, f64) {
        let (q, v) = self.world.state(self.joints[i]);
        self.transmissions[i].to_motor(q, v)
    }

    /// One step of physics, under the torques the motors give now.
    pub fn step(&mut self) {
        let now = self.world.time();
        for i in 0..self.motors.len() {
            let (q, v) = self.motor_state(i);
            let torque = self.motors[i].step(now, q, v);
            self.world.set_force(self.joints[i], self.transmissions[i].to_joint(torque));
        }
        self.world.step();
    }

    /// Whether every motor has had a frame since this last said yes: a
    /// cycle of the bus master, in lockstep.
    pub fn cycle_done(&mut self) -> bool {
        if self.heard.iter().all(|&h| h) {
            self.heard.fill(false);
            return true;
        }
        false
    }
}
