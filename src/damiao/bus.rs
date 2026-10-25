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
//! A motor set to another bit rate than the bus's (`CAN_BR`) neither hears
//! nor answers: on a real bus, it would only see errors. Nor does a motor
//! without the voltage to run.

use std::collections::HashSet;

use crate::can::Frame;
use crate::damiao::motor::Motor;
use crate::damiao::{bit_rate, mode, reg, Register, RegisterReply, REGISTER_ID};

pub struct Motors {
    motors: Vec<Motor>,
    can_fd: bool,
    /// The bus's bit rate, as `CAN_BR` codes it.
    bit_rate: u32,
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
    pub fn new(motors: Vec<Motor>, can_fd: bool, bit_rate: u32) -> Self {
        Self { motors, can_fd, bit_rate, said: HashSet::new() }
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
            return self.motors.iter().position(|m| m.id() == id);
        }
        self.motors.iter().position(|m| m.id() == frame.id).or_else(|| self.mode_frame(frame).map(|(i, _)| i))
    }

    /// The motor a command in another mode is for, and the mode.
    fn mode_frame(&self, frame: &Frame) -> Option<(usize, u32)> {
        self.motors.iter().enumerate().find_map(|(i, m)| {
            let offset = frame.id.checked_sub(m.id())?;
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
        let i = self.addressee(frame)?;
        if !self.motors[i].on() || self.motors[i].registers().bit_rate != self.bit_rate {
            return None;
        }
        let answer = if frame.id == REGISTER_ID {
            let asked = Register::decode(&frame.eight()?)?;
            let request = self.unshared(i, asked);
            let (q, v) = state(i);
            let answer = self.motors[i].on_register(request, now, q, v);
            if request == asked {
                self.explain(i, request, answer.as_ref());
            }
            answer
        } else if self.motors[i].id() == frame.id {
            let data = frame.eight()?;
            let (q, v) = state(i);
            let answer = self.motors[i].on_frame(&data, now, q, v);
            if answer.is_none() {
                self.wrong_mode(i, mode::MIT);
            }
            answer
        } else {
            let (_, mode) = self.mode_frame(frame)?;
            let (q, v) = state(i);
            let answer = self.motors[i].on_mode_frame(mode, frame.bytes(), now, q, v);
            if answer.is_none() {
                self.wrong_mode(i, mode);
            }
            answer
        };
        let can_fd = self.can_fd;
        Some(Delivery { motor: i, answer: answer.map(|a| a.fd(can_fd)) })
    }

    /// A write of another motor's id to motor `i`'s, refused: as a write of
    /// its own, which changes nothing and answers with it. Two real motors
    /// on one id would both take its frames and both answer; drydock
    /// doesn't simulate that.
    fn unshared(&mut self, i: usize, request: Register) -> Register {
        let Register::Write { motor, rid: reg::ESC_ID, value } = request else { return request };
        let id = u32::from_le_bytes(value);
        let Some(other) = self.motors.iter().enumerate().find(|&(j, m)| j != i && m.id() as u32 == id) else {
            return request;
        };
        let message = format!(
            "{}: ESC_ID {id:#x} is {}'s: refused, as drydock doesn't simulate two motors on one id",
            self.motors[i].joint, other.1.joint
        );
        self.say(message);
        Register::Write { motor, rid: reg::ESC_ID, value: (self.motors[i].id() as u32).to_le_bytes() }
    }

    /// Says why a register request didn't do what it asked, once.
    fn explain(&mut self, i: usize, request: Register, answer: Option<&Frame>) {
        let joint = self.motors[i].joint.clone();
        match (request, answer) {
            (Register::Read { rid, .. } | Register::Write { rid, .. }, None) if !reg::exists(rid) => {
                self.say(format!("{joint}: there's no register {rid} in Damiao's table: no answer"))
            }
            (Register::Read { rid, .. } | Register::Write { rid, .. }, None) => self.say(format!(
                "{joint}: register {rid}'s value isn't published, and drydock has no model for it: no answer until it's written"
            )),
            (Register::Write { rid: reg::CAN_BR, .. }, Some(_))
                if self.motors[i].registers().bit_rate != self.bit_rate =>
            {
                let message = format!(
                    "{joint}: now at {} (CAN_BR {}), on a bus at {}: it neither hears nor answers until drydock restarts",
                    bit_rate::name(self.motors[i].registers().bit_rate),
                    self.motors[i].registers().bit_rate,
                    bit_rate::name(self.bit_rate)
                );
                self.say(message)
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
