//! The Damiao motors on one CAN bus: whose a frame is, and what answers it.
//!
//! A frame to a motor's id is that motor's; a frame to `0x7FF` is for the
//! motor named in its first two bytes; frames to `id + 0x100`, `+ 0x200` and
//! `+ 0x300` are its commands in its other modes. A command for another
//! mode than the motor's is ignored, with a note.
//!
//! Motors on CAN FD answer in it and understand classic frames too, as CAN
//! FD controllers do. Motors on classic CAN don't understand CAN FD (on a
//! real bus, they'd answer with error frames): it's ignored, with a note.

use std::collections::HashSet;

use crate::can::Frame;
use crate::damiao::motor::Motor;
use crate::damiao::{mode, Register, RegisterReply, REGISTER_ID};

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

    /// Which motor a frame is for, by its place on the bus, if any.
    pub fn addressee(&self, frame: &Frame) -> Option<usize> {
        if frame.id == REGISTER_ID {
            let id = Register::decode(&frame.eight()?)?.motor();
            return self.motors.iter().position(|m| m.id == id);
        }
        self.motors.iter().position(|m| m.id == frame.id).or_else(|| self.mode_frame(frame).map(|(i, _)| i))
    }

    /// The motor a command in another mode is for, and the mode.
    fn mode_frame(&self, frame: &Frame) -> Option<(usize, u32)> {
        self.motors.iter().enumerate().find_map(|(i, m)| {
            let offset = frame.id.checked_sub(m.id)?;
            mode::of_offset(offset).map(|mode| (i, mode))
        })
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
        let (motor, answer) = if frame.id == REGISTER_ID {
            let request = Register::decode(&frame.eight()?)?;
            let i = self.motors.iter().position(|m| m.id == request.motor())?;
            let (q, v) = state(i);
            let answer = self.motors[i].on_register(request, now, q, v);
            self.explain(i, request, answer.as_ref());
            (i, answer)
        } else if let Some(i) = self.motors.iter().position(|m| m.id == frame.id) {
            let (q, v) = state(i);
            let answer = self.motors[i].on_frame(&frame.eight()?, now, q, v);
            if answer.is_none() {
                self.wrong_mode(i, mode::MIT);
            }
            (i, answer)
        } else if let Some((i, mode)) = self.mode_frame(frame) {
            let (q, v) = state(i);
            let answer = self.motors[i].on_mode_frame(mode, frame.bytes(), now, q, v);
            if answer.is_none() {
                self.wrong_mode(i, mode);
            }
            (i, answer)
        } else {
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

    /// Says, once, that motor `i` got a command for another mode than its own.
    fn wrong_mode(&mut self, i: usize, sent: u32) {
        let m = &self.motors[i];
        let message = format!(
            "{}: a {} command, but the motor is in {} mode (CTRL_MODE {}): ignored",
            m.joint,
            mode::name(sent),
            mode::name(m.mode()),
            m.mode()
        );
        self.say(message);
    }

    fn say(&mut self, message: String) {
        if self.said.insert(message.clone()) {
            eprintln!("drydock: {message}");
        }
    }
}
