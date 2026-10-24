//! One Damiao motor in MIT mode: what it does with the frames it gets, and
//! the torque it puts on its joint. It knows nothing of MuJoCo: it is handed
//! its joint's position and velocity, and the time.

use crate::can::Frame;
use crate::damiao::{reg, status, Feedback, Limits, Mit, Register, RegisterReply, Special};

/// What drydock reports for both temperatures: it has no thermal model.
pub const TEMPERATURE: u8 = 25;

#[derive(Debug, Clone)]
pub struct Motor {
    /// The joint it drives, in the scene.
    pub joint: String,
    /// Its CAN id (ESC_ID), where commands go.
    pub id: u16,
    /// Where its answers go (MST_ID).
    pub master_id: u16,
    /// PMAX, VMAX, TMAX: writable through registers.
    pub limits: Limits,
    /// The most torque it gives, N m.
    pub peak_torque: f64,
    /// Without a frame for this long, seconds, it disables itself with
    /// [`status::COMMUNICATION_LOST`].
    pub timeout: Option<f64>,
    status: u8,
    /// The command in force while enabled.
    command: Mit,
    /// When it last got a frame.
    heard: f64,
    /// The joint's position that reads as zero.
    zero: f64,
    /// The torque it gave on the last step.
    torque: f64,
}

impl Motor {
    pub fn new(joint: String, id: u16, master_id: u16, limits: Limits, peak_torque: f64, timeout: Option<f64>) -> Self {
        Self {
            joint,
            id,
            master_id,
            limits,
            peak_torque,
            timeout,
            status: status::DISABLED,
            command: Mit::default(),
            heard: 0.0,
            zero: 0.0,
            torque: 0.0,
        }
    }

    pub fn status(&self) -> u8 {
        self.status
    }

    pub fn torque(&self) -> f64 {
        self.torque
    }

    /// A frame to its id: a command, or one of the special frames. Answered
    /// with feedback, as a real motor answers each.
    ///
    /// A command counts only while enabled; enabling starts from no command
    /// (zero gains, zero torque) until the next one. A motor in error stays
    /// in it, enabled or not, until the error is cleared.
    pub fn on_frame(&mut self, data: &[u8; 8], now: f64, q: f64, v: f64) -> Frame {
        self.heard = now;
        match Special::decode(data) {
            Some(Special::Enable) => {
                if self.status == status::DISABLED {
                    self.status = status::ENABLED;
                    self.command = Mit::default();
                }
            }
            Some(Special::Disable) => {
                if self.status == status::ENABLED {
                    self.status = status::DISABLED;
                }
            }
            Some(Special::Zero) => self.zero = q,
            Some(Special::ClearError) => {
                if !matches!(self.status, status::DISABLED | status::ENABLED) {
                    self.status = status::DISABLED;
                }
            }
            None => {
                if self.status == status::ENABLED {
                    self.command = Mit::decode(data, self.limits);
                }
            }
        }
        if self.status != status::ENABLED {
            self.torque = 0.0;
        }
        self.feedback(q, v)
    }

    /// A register request for this motor. Reads and writes are answered
    /// with the register's value after them, so a write it refused answers
    /// with the value it kept. A register it doesn't simulate gets no
    /// answer. Save answers nothing; refresh, with feedback.
    pub fn on_register(&mut self, request: Register, now: f64, q: f64, v: f64) -> Option<Frame> {
        self.heard = now;
        let (rid, write) = match request {
            Register::Refresh { .. } => return Some(self.feedback(q, v)),
            Register::Save { .. } => return None,
            Register::Read { rid, .. } => (rid, false),
            Register::Write { rid, value, .. } => {
                self.write(rid, value);
                (rid, true)
            }
        };
        let value = self.register(rid)?;
        Some(Frame::new(self.master_id, RegisterReply { motor: self.id, write, rid, value }.encode()))
    }

    /// A register's value, if simulated.
    pub fn register(&self, rid: u8) -> Option<[u8; 4]> {
        Some(match rid {
            reg::MST_ID => (self.master_id as u32).to_le_bytes(),
            reg::ESC_ID => (self.id as u32).to_le_bytes(),
            reg::CTRL_MODE => reg::MIT_MODE.to_le_bytes(),
            reg::PMAX => self.limits.p.to_le_bytes(),
            reg::VMAX => self.limits.v.to_le_bytes(),
            reg::TMAX => self.limits.t.to_le_bytes(),
            _ => return None,
        })
    }

    /// The ranges are writable. The ids aren't, since the bus would have to
    /// change under drydock; nor is the mode, since only MIT mode is
    /// simulated.
    fn write(&mut self, rid: u8, value: [u8; 4]) {
        let x = f32::from_le_bytes(value);
        if !(x.is_finite() && x > 0.0) {
            return;
        }
        match rid {
            reg::PMAX => self.limits.p = x,
            reg::VMAX => self.limits.v = x,
            reg::TMAX => self.limits.t = x,
            _ => {}
        }
    }

    /// The torque on its joint until the next step: the MIT law on its
    /// command while enabled, within its peak torque; none otherwise.
    pub fn step(&mut self, now: f64, q: f64, v: f64) -> f64 {
        if self.status == status::ENABLED && self.timeout.is_some_and(|t| now - self.heard > t) {
            self.status = status::COMMUNICATION_LOST;
        }
        self.torque = if self.status == status::ENABLED {
            let c = self.command;
            let p = q - self.zero;
            let t = c.kp as f64 * (c.p as f64 - p) + c.kd as f64 * (c.v as f64 - v) + c.t as f64;
            t.clamp(-self.peak_torque, self.peak_torque)
        } else {
            0.0
        };
        self.torque
    }

    fn feedback(&self, q: f64, v: f64) -> Frame {
        let f = Feedback {
            id: (self.id & 0xF) as u8,
            status: self.status,
            p: (q - self.zero) as f32,
            v: v as f32,
            t: self.torque as f32,
            t_mos: TEMPERATURE,
            t_rotor: TEMPERATURE,
        };
        Frame::new(self.master_id, f.encode(self.limits))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::damiao::Model;

    fn motor() -> Motor {
        Motor::new("j".into(), 1, 0x11, Model::DM4310.limits(), 7.0, Some(0.1))
    }

    fn command(m: &Motor, c: Mit) -> [u8; 8] {
        c.encode(m.limits)
    }

    #[test]
    fn applies_the_mit_law_only_while_enabled() {
        let mut m = motor();
        let c = command(&m, Mit { p: 1.0, v: 0.0, kp: 2.0, kd: 0.5, t: 0.25 });
        m.on_frame(&c, 0.0, 0.0, 0.0);
        assert_eq!(m.step(0.0, 0.0, 0.0), 0.0);
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.on_frame(&c, 0.0, 0.0, 0.0);
        let t = m.step(0.0, 0.5, 1.0);
        // The law on the command as packed: kp has 12 bits over 0..500.
        let d = Mit::decode(&c, m.limits);
        let law = d.kp as f64 * (d.p as f64 - 0.5) + d.kd as f64 * (d.v as f64 - 1.0) + d.t as f64;
        assert!((t - law).abs() < 1e-9, "{t} {law}");
        assert!((t - 0.75).abs() < 0.1, "{t}");
    }

    #[test]
    fn torque_stays_within_its_peak() {
        let mut m = motor();
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.on_frame(&command(&m, Mit { p: 10.0, kp: 500.0, ..Default::default() }), 0.0, 0.0, 0.0);
        assert_eq!(m.step(0.0, 0.0, 0.0), 7.0);
    }

    #[test]
    fn lost_commands_disable_it_until_the_error_is_cleared() {
        let mut m = motor();
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.step(0.05, 0.0, 0.0);
        assert_eq!(m.status(), status::ENABLED);
        m.step(0.2, 0.0, 0.0);
        assert_eq!(m.status(), status::COMMUNICATION_LOST);
        m.on_frame(&Special::Enable.encode(), 0.2, 0.0, 0.0);
        assert_eq!(m.status(), status::COMMUNICATION_LOST);
        m.on_frame(&Special::ClearError.encode(), 0.2, 0.0, 0.0);
        assert_eq!(m.status(), status::DISABLED);
        let f = m.on_frame(&Special::Enable.encode(), 0.2, 0.0, 0.0);
        assert_eq!(f.data[0] >> 4, status::ENABLED);
    }

    #[test]
    fn zero_moves_where_positions_count_from() {
        let mut m = motor();
        m.on_frame(&Special::Zero.encode(), 0.0, 1.5, 0.0);
        let f = m.on_frame(&Special::Enable.encode(), 0.0, 1.75, 0.0);
        assert!((Feedback::decode(&f.eight().unwrap(), m.limits).p - 0.25).abs() < 1e-3);
    }

    #[test]
    fn registers() {
        let mut m = motor();
        let read = |m: &mut Motor, rid| m.on_register(Register::Read { motor: 1, rid }, 0.0, 0.0, 0.0);
        let f = read(&mut m, reg::PMAX).unwrap();
        assert_eq!(f.id, 0x11);
        assert_eq!(RegisterReply::decode(&f.eight().unwrap()).unwrap().value, 12.5f32.to_le_bytes());
        assert!(read(&mut m, reg::TIMEOUT).is_none());

        let write = Register::Write { motor: 1, rid: reg::CTRL_MODE, value: 2u32.to_le_bytes() };
        let reply = RegisterReply::decode(&m.on_register(write, 0.0, 0.0, 0.0).unwrap().eight().unwrap()).unwrap();
        assert!(reply.write);
        assert_eq!(reply.value, reg::MIT_MODE.to_le_bytes());

        let write = Register::Write { motor: 1, rid: reg::TMAX, value: 20.0f32.to_le_bytes() };
        m.on_register(write, 0.0, 0.0, 0.0);
        assert_eq!(m.limits.t, 20.0);
    }
}
