//! The Damiao motors on one CAN bus: whose a frame is, and what answers it.
//!
//! A frame to a motor's id is that motor's; a frame to `0x7FF` is for the
//! motor named in its first two bytes; frames to `id + 0x100`, `+ 0x200` and
//! `+ 0x300` are other modes than MIT, noted and ignored.
//!
//! Motors on CAN FD answer in it and understand classic frames too, as CAN
//! FD controllers do. Motors on classic CAN don't understand CAN FD (on a
//! real bus, they'd answer with error frames): it's ignored, with a note.

use std::collections::HashSet;

use crate::can::Frame;
use crate::damiao::motor::Motor;
use crate::damiao::{Register, RegisterReply, REGISTER_ID};

pub struct Motors {
    motors: Vec<Motor>,
    can_fd: bool,
    /// What has been said once, not to say it again.
    said: HashSet<String>,
}

/// A frame that reached a motor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Delivery {
    /// Which motor, by its place on the bus.
    pub motor: usize,
    pub answer: Option<Frame>,
}

impl Motors {
    pub fn new(motors: Vec<Motor>, can_fd: bool) -> Self {
        Self { motors, can_fd, said: HashSet::new() }
    }

    pub fn motors(&self) -> &[Motor] {
        &self.motors
    }

    pub fn motor_mut(&mut self, i: usize) -> &mut Motor {
        &mut self.motors[i]
    }

    pub fn can_fd(&self) -> bool {
        self.can_fd
    }

    /// A frame reached the motors at `now`: which one it was for, and its
    /// answer. `state(i)` is motor `i`'s angle and velocity, as it senses
    /// them.
    pub fn deliver(&mut self, frame: &Frame, now: f64, mut state: impl FnMut(usize) -> (f64, f64)) -> Option<Delivery> {
        if frame.fd && !self.can_fd {
            self.say(format!(
                "a CAN FD frame to {:#05x}, but the motors are on classic CAN (can_fd: false): ignored",
                frame.id
            ));
            return None;
        }
        let data = frame.eight()?;
        let (motor, answer) = if frame.id == REGISTER_ID {
            let request = Register::decode(&data)?;
            let i = self.motors.iter().position(|m| m.id == request.motor())?;
            let (q, v) = state(i);
            let answer = self.motors[i].on_register(request, now, q, v);
            self.explain(i, request, answer.as_ref());
            (i, answer)
        } else if let Some(i) = self.motors.iter().position(|m| m.id == frame.id) {
            let (q, v) = state(i);
            (i, Some(self.motors[i].on_frame(&data, now, q, v)))
        } else {
            if let Some(m) = self.motors.iter().find(|m| [0x100, 0x200, 0x300].iter().any(|&o| frame.id == m.id + o)) {
                let joint = m.joint.clone();
                self.say(format!("{joint}: a frame for another mode than MIT, which is all drydock simulates"));
            }
            return None;
        };
        let can_fd = self.can_fd;
        Some(Delivery { motor, answer: answer.map(|a| a.fd(can_fd)) })
    }

    /// Says why a register request didn't do what it asked, once.
    fn explain(&mut self, i: usize, request: Register, answer: Option<&Frame>) {
        let joint = self.motors[i].joint.clone();
        match (request, answer) {
            (Register::Read { rid, .. } | Register::Write { rid, .. }, None) => {
                self.say(format!("{joint}: register {rid} isn't simulated: no answer"))
            }
            (Register::Write { rid, value, .. }, Some(answer))
                if answer.eight().and_then(|d| RegisterReply::decode(&d)).is_some_and(|r| r.value != value) =>
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
}
