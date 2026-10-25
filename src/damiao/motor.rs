//! One Damiao motor, as its firmware behaves: what it does with the frames
//! it gets, and the torque its control law asks for, in MIT mode or one of
//! its other modes (position-velocity, velocity, position-force). It knows
//! nothing of MuJoCo, nor of what limits the torque it gets
//! (`drivetrain.rs`): it is handed its own angle and velocity, the time, the
//! torque it was given, and how hot it is (`thermal.rs`).

use crate::can::Frame;
use crate::damiao::{
    mode, reg, status, Feedback, Limits, Mit, PosForce, PosVel, Register, RegisterReply, Special, Vel,
};

/// The over-temperature threshold a motor starts with, °C. Damiao doesn't
/// say what theirs leave the factory with; assumed.
pub const OVER_TEMPERATURE: f32 = 120.0;

/// The motor's own loops in its other modes: a speed loop, and a position
/// loop around it. Damiao sets their gains with registers (`KP_ASR`,
/// `KI_ASR`, `KP_APR`) but doesn't publish their defaults, so these are
/// assumed, and chosen to match the MIT gains OpenArm's driver gives its
/// motors (kd about TMAX / 20, kp about 20 kd).
///
/// The speed loop's gain: torque per rad/s of error, as a share of TMAX.
const SPEED_GAIN: f64 = 1.0 / 20.0;
/// How fast its integral catches up, per second: what holds a load.
const SPEED_INTEGRAL: f64 = 10.0;
/// The position loop's: rad/s asked per rad of error.
const POSITION_GAIN: f64 = 20.0;

/// The command in force, in the mode it was given in.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
enum Command {
    /// Enabled, or switched modes, and told nothing since: no torque.
    #[default]
    None,
    Mit(Mit),
    PosVel(PosVel),
    Vel(Vel),
    PosForce(PosForce),
}

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
    /// Without a frame for this long, seconds, it disables itself with
    /// [`status::COMMUNICATION_LOST`].
    pub timeout: Option<f64>,
    status: u8,
    /// CTRL_MODE: which commands it takes.
    mode: u32,
    /// The command in force while enabled.
    command: Command,
    /// The speed loop's integral, N m, and when the law last ran.
    integral: f64,
    last_law: Option<f64>,
    /// When it last got a frame.
    heard: f64,
    /// The joint's position that reads as zero.
    zero: f64,
    /// The torque it got on the last step, which it reports.
    torque: f64,
    /// OT_VALUE, °C.
    over_temperature: f32,
    /// Its drive's and its coil's temperatures, °C, which it reports.
    drive_c: f64,
    coil_c: f64,
}

impl Motor {
    pub fn new(joint: String, id: u16, master_id: u16, limits: Limits, timeout: Option<f64>) -> Self {
        Self {
            joint,
            id,
            master_id,
            limits,
            timeout,
            status: status::DISABLED,
            mode: mode::MIT,
            command: Command::None,
            integral: 0.0,
            last_law: None,
            heard: 0.0,
            zero: 0.0,
            torque: 0.0,
            over_temperature: OVER_TEMPERATURE,
            drive_c: crate::thermal::AMBIENT,
            coil_c: crate::thermal::AMBIENT,
        }
    }

    pub fn status(&self) -> u8 {
        self.status
    }

    pub fn torque(&self) -> f64 {
        self.torque
    }

    /// The mode it's in.
    pub fn mode(&self) -> u32 {
        self.mode
    }

    /// A frame to its id: one of the special frames, in any mode, or an MIT
    /// command, in MIT mode. Answered with feedback, as a real motor answers
    /// each; an MIT command in another mode is ignored, unanswered.
    ///
    /// A command counts only while enabled; enabling starts from no command
    /// (no torque) until the next one. A motor in error stays in it, enabled
    /// or not, until the error is cleared.
    pub fn on_frame(&mut self, data: &[u8; 8], now: f64, q: f64, v: f64) -> Option<Frame> {
        self.heard = now;
        match Special::decode(data) {
            Some(Special::Enable) => {
                if self.status == status::DISABLED {
                    self.status = status::ENABLED;
                    self.reset();
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
            None if self.mode != mode::MIT => return None,
            None => {
                if self.status == status::ENABLED {
                    self.command = Command::Mit(Mit::decode(data, self.limits));
                }
            }
        }
        if self.status != status::ENABLED {
            self.torque = 0.0;
        }
        Some(self.feedback(q, v))
    }

    /// A command in another mode than MIT, to `id + 0x100`, `+ 0x200` or
    /// `+ 0x300`: taken while enabled in that mode, and answered with
    /// feedback. In another mode, or malformed, it's ignored, unanswered.
    pub fn on_mode_frame(&mut self, mode: u32, data: &[u8], now: f64, q: f64, v: f64) -> Option<Frame> {
        if mode != self.mode {
            return None;
        }
        let command = match mode {
            mode::POS_VEL => Command::PosVel(PosVel::decode(data)?),
            mode::VEL => Command::Vel(Vel::decode(data)?),
            mode::POS_FORCE => Command::PosForce(PosForce::decode(data)?),
            _ => return None,
        };
        self.heard = now;
        if self.status == status::ENABLED {
            self.command = command;
        } else {
            self.torque = 0.0;
        }
        Some(self.feedback(q, v))
    }

    /// No command, and the speed loop starting afresh.
    fn reset(&mut self) {
        self.command = Command::None;
        self.integral = 0.0;
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
            reg::OT_VALUE => self.over_temperature.to_le_bytes(),
            reg::MST_ID => (self.master_id as u32).to_le_bytes(),
            reg::ESC_ID => (self.id as u32).to_le_bytes(),
            reg::CTRL_MODE => self.mode.to_le_bytes(),
            reg::PMAX => self.limits.p.to_le_bytes(),
            reg::VMAX => self.limits.v.to_le_bytes(),
            reg::TMAX => self.limits.t.to_le_bytes(),
            _ => return None,
        })
    }

    /// The ranges, the over-temperature threshold within 80 to 200 °C, and
    /// the mode (1 to 4) are writable. A new mode starts from no command.
    /// The ids aren't, since the bus would have to change under drydock.
    fn write(&mut self, rid: u8, value: [u8; 4]) {
        if rid == reg::CTRL_MODE {
            let m = u32::from_le_bytes(value);
            if (mode::MIT..=mode::POS_FORCE).contains(&m) && m != self.mode {
                self.mode = m;
                self.reset();
            }
            return;
        }
        let x = f32::from_le_bytes(value);
        if !(x.is_finite() && x > 0.0) {
            return;
        }
        match rid {
            reg::OT_VALUE if (80.0..200.0).contains(&x) => self.over_temperature = x,
            reg::PMAX => self.limits.p = x,
            reg::VMAX => self.limits.v = x,
            reg::TMAX => self.limits.t = x,
            _ => {}
        }
    }

    /// The torque it asks for until the next step, while enabled: the MIT
    /// law on an MIT command; in the other modes, its own speed loop, and a
    /// position loop around it for position-velocity and position-force.
    /// None otherwise. A motor past its timeout disables itself here.
    pub fn law(&mut self, now: f64, q: f64, v: f64) -> f64 {
        let dt = self.last_law.map_or(0.0, |t| (now - t).max(0.0));
        self.last_law = Some(now);
        if self.status == status::ENABLED && self.timeout.is_some_and(|t| now - self.heard > t) {
            self.status = status::COMMUNICATION_LOST;
        }
        if self.status != status::ENABLED {
            return 0.0;
        }
        let p = q - self.zero;
        let tmax = self.limits.t as f64;
        let toward = |target: f32, speed: f32| {
            (POSITION_GAIN * (target as f64 - p)).clamp(-speed.abs() as f64, speed.abs() as f64)
        };
        match self.command {
            Command::None => 0.0,
            Command::Mit(c) => c.kp as f64 * (c.p as f64 - p) + c.kd as f64 * (c.v as f64 - v) + c.t as f64,
            Command::Vel(c) => self.speed(c.v as f64, v, dt, tmax),
            Command::PosVel(c) => self.speed(toward(c.p, c.v), v, dt, tmax),
            // The current's share of its maximum, taken as the torque's of TMAX.
            Command::PosForce(c) => self.speed(toward(c.p, c.v), v, dt, c.i as f64 * tmax),
        }
    }

    /// The speed loop: the torque that turns it at `wanted` (rad/s), turning
    /// at `v`, at most `limit` (N m) either way.
    fn speed(&mut self, wanted: f64, v: f64, dt: f64, limit: f64) -> f64 {
        let gain = SPEED_GAIN * self.limits.t as f64;
        let error = wanted - v;
        self.integral = (self.integral + SPEED_INTEGRAL * gain * error * dt).clamp(-limit, limit);
        (gain * error + self.integral).clamp(-limit, limit)
    }

    /// The torque it got, after what limits it: what it reports.
    pub fn applied(&mut self, torque: f64) {
        self.torque = torque;
    }

    /// How hot its drive and its coil are, °C. Past its threshold while
    /// enabled, it stops with an over-temperature error, the drive's first,
    /// until cleared. There's no hysteresis in Damiao's sources, so none
    /// here: cleared and enabled, it runs until it's past again.
    pub fn temperatures(&mut self, drive: f64, coil: f64) {
        (self.drive_c, self.coil_c) = (drive, coil);
        if self.status != status::ENABLED {
            return;
        }
        let limit = self.over_temperature as f64;
        if drive >= limit {
            self.status = status::MOS_OVERTEMPERATURE;
        } else if coil >= limit {
            self.status = status::COIL_OVERTEMPERATURE;
        }
    }

    fn feedback(&self, q: f64, v: f64) -> Frame {
        let f = Feedback {
            id: (self.id & 0xF) as u8,
            status: self.status,
            p: (q - self.zero) as f32,
            v: v as f32,
            t: self.torque as f32,
            t_mos: self.drive_c.round().clamp(0.0, 255.0) as u8,
            t_rotor: self.coil_c.round().clamp(0.0, 255.0) as u8,
        };
        Frame::new(self.master_id, f.encode(self.limits))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::damiao::Model;

    fn motor() -> Motor {
        Motor::new("j".into(), 1, 0x11, Model::DM4310.limits(), Some(0.1))
    }

    fn command(m: &Motor, c: Mit) -> [u8; 8] {
        c.encode(m.limits)
    }

    #[test]
    fn applies_the_mit_law_only_while_enabled() {
        let mut m = motor();
        let c = command(&m, Mit { p: 1.0, v: 0.0, kp: 2.0, kd: 0.5, t: 0.25 });
        m.on_frame(&c, 0.0, 0.0, 0.0);
        assert_eq!(m.law(0.0, 0.0, 0.0), 0.0);
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.on_frame(&c, 0.0, 0.0, 0.0);
        let t = m.law(0.0, 0.5, 1.0);
        // The law on the command as packed: kp has 12 bits over 0..500.
        let d = Mit::decode(&c, m.limits);
        let law = d.kp as f64 * (d.p as f64 - 0.5) + d.kd as f64 * (d.v as f64 - 1.0) + d.t as f64;
        assert!((t - law).abs() < 1e-9, "{t} {law}");
        assert!((t - 0.75).abs() < 0.1, "{t}");
    }

    #[test]
    fn reports_the_torque_it_got() {
        let mut m = motor();
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.applied(-1.5);
        let f = m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0).unwrap();
        assert!((Feedback::decode(&f.eight().unwrap(), m.limits).t + 1.5).abs() < 0.01);
    }

    #[test]
    fn lost_commands_disable_it_until_the_error_is_cleared() {
        let mut m = motor();
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.law(0.05, 0.0, 0.0);
        assert_eq!(m.status(), status::ENABLED);
        m.law(0.2, 0.0, 0.0);
        assert_eq!(m.status(), status::COMMUNICATION_LOST);
        m.on_frame(&Special::Enable.encode(), 0.2, 0.0, 0.0);
        assert_eq!(m.status(), status::COMMUNICATION_LOST);
        m.on_frame(&Special::ClearError.encode(), 0.2, 0.0, 0.0);
        assert_eq!(m.status(), status::DISABLED);
        let f = m.on_frame(&Special::Enable.encode(), 0.2, 0.0, 0.0).unwrap();
        assert_eq!(f.data[0] >> 4, status::ENABLED);
    }

    #[test]
    fn zero_moves_where_positions_count_from() {
        let mut m = motor();
        m.on_frame(&Special::Zero.encode(), 0.0, 1.5, 0.0);
        let f = m.on_frame(&Special::Enable.encode(), 0.0, 1.75, 0.0).unwrap();
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

        let write = |m: &mut Motor, mode: u32| {
            let w = Register::Write { motor: 1, rid: reg::CTRL_MODE, value: mode.to_le_bytes() };
            RegisterReply::decode(&m.on_register(w, 0.0, 0.0, 0.0).unwrap().eight().unwrap()).unwrap()
        };
        let reply = write(&mut m, mode::VEL);
        assert!(reply.write);
        assert_eq!(reply.value, mode::VEL.to_le_bytes());
        // No fifth mode: refused, and the answer says which it kept.
        assert_eq!(write(&mut m, 5).value, mode::VEL.to_le_bytes());

        let write = Register::Write { motor: 1, rid: reg::TMAX, value: 20.0f32.to_le_bytes() };
        m.on_register(write, 0.0, 0.0, 0.0);
        assert_eq!(m.limits.t, 20.0);
    }
}
