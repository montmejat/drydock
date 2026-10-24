//! A simulated robot: the world, and the motors on its joints. Frames in,
//! frames out, and steps of physics; how time passes is up to the caller
//! (`run.rs`).

use std::collections::HashSet;

use crate::can::Frame;
use crate::damiao::{Register, RegisterReply, REGISTER_ID};
use crate::motor::Motor;
use crate::robot::Robot;
use crate::world::{Joint, World};

pub struct Drydock {
    world: World,
    motors: Vec<Motor>,
    joints: Vec<Joint>,
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
        for spec in &robot.motors {
            let joint = world.joint(&spec.joint)?;
            let released = world.release_actuators(joint);
            if released > 0 {
                eprintln!("drydock: the scene's actuator(s) on `{}` are off: its motor drives it", spec.joint);
            }
            let limits = spec.model.limits();
            let peak = spec.peak_torque.unwrap_or(limits.t as f64);
            let timeout = spec.timeout_ms.map(|ms| ms / 1e3);
            motors.push(Motor::new(spec.joint.clone(), spec.id, spec.master_id, limits, peak, timeout));
            joints.push(joint);
        }
        let heard = vec![false; motors.len()];
        Ok(Self { world, motors, joints, heard, said: HashSet::new() })
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

    /// A frame from the bus, and the motor's answer if one answers.
    pub fn handle(&mut self, frame: &Frame) -> Option<Frame> {
        if frame.len != 8 {
            return None;
        }
        let now = self.world.time();
        if frame.id == REGISTER_ID {
            let request = Register::decode(&frame.data)?;
            let i = self.motors.iter().position(|m| m.id == request.motor())?;
            self.heard[i] = true;
            let (q, v) = self.world.state(self.joints[i]);
            let reply = self.motors[i].on_register(request, now, q, v);
            self.explain(i, request, reply.as_ref());
            return reply;
        }
        if let Some(i) = self.motors.iter().position(|m| m.id == frame.id) {
            self.heard[i] = true;
            let (q, v) = self.world.state(self.joints[i]);
            return Some(self.motors[i].on_frame(&frame.data, now, q, v));
        }
        // Position-velocity mode is at id + 0x100, velocity mode at + 0x200.
        if let Some(m) = self.motors.iter().find(|m| frame.id == m.id + 0x100 || frame.id == m.id + 0x200) {
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
                if RegisterReply::decode(&reply.data).is_some_and(|r| r.value != value) =>
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

    /// One step of physics, under the torques the motors give now.
    pub fn step(&mut self) {
        let now = self.world.time();
        for (motor, &joint) in self.motors.iter_mut().zip(&self.joints) {
            let (q, v) = self.world.state(joint);
            let torque = motor.step(now, q, v);
            self.world.set_force(joint, torque);
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
