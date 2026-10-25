//! One Damiao motor, as its firmware behaves: what it does with the frames
//! it gets, and the torque its control law asks for, in MIT mode or one of
//! its other modes (position-velocity, velocity, position-force). It knows
//! nothing of MuJoCo, nor of what limits the torque it gets
//! (`drivetrain.rs`): it is handed its own angle and velocity, the time, the
//! torque it was given, and how hot it is (`thermal.rs`).

use std::collections::BTreeMap;

use crate::can::Frame;
use crate::damiao::{
    mode, reg, status, Feedback, Limits, Mit, PosForce, PosVel, Register, RegisterReply, Saved, Special, Vel,
};

/// The over-temperature threshold a motor starts with, °C. Damiao doesn't
/// say what theirs leave the factory with; assumed.
pub const OVER_TEMPERATURE: f32 = 120.0;

/// The undervoltage threshold a motor starts with, V: the least Damiao's
/// manual recommends. What they leave the factory with isn't published.
pub const UNDER_VOLTAGE: f32 = 15.0;

/// The overvoltage threshold a motor starts with, V: the most Damiao's
/// manual recommends, for a motor run at `volts`. What they leave the
/// factory with isn't published.
pub fn over_voltage(volts: f64) -> f32 {
    if volts > 24.0 {
        52.0
    } else {
        32.0
    }
}

/// Below this, V, a motor is off: it neither hears nor answers. Assumed:
/// the least `UV_Value` may be set to.
pub const POWER_ON: f64 = 10.0;

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

/// What a motor holds in its registers: what writes through `0x7FF` change,
/// at once.
#[derive(Debug, Clone, PartialEq)]
pub struct Registers {
    /// ESC_ID: where its commands go.
    pub id: u16,
    /// MST_ID: where its answers go.
    pub master_id: u16,
    /// PMAX, VMAX, TMAX.
    pub limits: Limits,
    /// TIMEOUT, in [`reg::TIMEOUT_UNIT`]: without a frame for this long, it
    /// stops with [`status::COMMUNICATION_LOST`]. 0 for never.
    pub timeout: u32,
    /// CAN_BR: the bit rate it runs its bus at, as a code.
    pub bit_rate: u32,
    /// CTRL_MODE: which commands it takes.
    pub mode: u32,
    /// OT_VALUE, °C.
    pub over_temperature: f32,
    /// UV_VALUE and OV_VALUE, V.
    pub under_voltage: f32,
    pub over_voltage: f32,
    /// The rest of Damiao's table, once written. Nothing in drydock uses
    /// them, and their factory values aren't published, so until written
    /// they get no answer rather than a made-up one.
    others: BTreeMap<u8, [u8; 4]>,
}

impl Registers {
    /// As a motor leaves the factory, at these ids and ranges, made to run
    /// at `volts`: classic CAN at 1 Mbit/s (Damiao's manual), MIT mode, no
    /// timeout.
    pub fn new(id: u16, master_id: u16, limits: Limits, volts: f64) -> Self {
        Self {
            id,
            master_id,
            limits,
            timeout: 0,
            bit_rate: 4,
            mode: mode::MIT,
            over_temperature: OVER_TEMPERATURE,
            under_voltage: UNDER_VOLTAGE,
            over_voltage: over_voltage(volts),
            others: BTreeMap::new(),
        }
    }

    /// The timeout, seconds, if it has one.
    pub fn timeout(&self) -> Option<f64> {
        (self.timeout > 0).then_some(self.timeout as f64 * reg::TIMEOUT_UNIT)
    }

    /// Its value, if drydock knows it.
    fn get(&self, rid: u8) -> Option<[u8; 4]> {
        Some(match rid {
            reg::MST_ID => (self.master_id as u32).to_le_bytes(),
            reg::ESC_ID => (self.id as u32).to_le_bytes(),
            reg::TIMEOUT => self.timeout.to_le_bytes(),
            reg::CTRL_MODE => self.mode.to_le_bytes(),
            reg::CAN_BR => self.bit_rate.to_le_bytes(),
            reg::OT_VALUE => self.over_temperature.to_le_bytes(),
            reg::UV_VALUE => self.under_voltage.to_le_bytes(),
            reg::OV_VALUE => self.over_voltage.to_le_bytes(),
            reg::PMAX => self.limits.p.to_le_bytes(),
            reg::VMAX => self.limits.v.to_le_bytes(),
            reg::TMAX => self.limits.t.to_le_bytes(),
            _ => return self.others.get(&rid).copied(),
        })
    }

    /// A value [`reg::takes`] said it takes.
    fn set(&mut self, rid: u8, value: [u8; 4]) {
        let (x, n) = (f32::from_le_bytes(value), u32::from_le_bytes(value));
        match rid {
            reg::MST_ID => self.master_id = n as u16,
            reg::ESC_ID => self.id = n as u16,
            reg::TIMEOUT => self.timeout = n,
            reg::CTRL_MODE => self.mode = n,
            reg::CAN_BR => self.bit_rate = n,
            reg::OT_VALUE => self.over_temperature = x,
            reg::UV_VALUE => self.under_voltage = x,
            reg::OV_VALUE => self.over_voltage = x,
            reg::PMAX => self.limits.p = x,
            reg::VMAX => self.limits.v = x,
            reg::TMAX => self.limits.t = x,
            _ => {
                self.others.insert(rid, value);
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct Motor {
    /// The joint it drives, in the scene.
    pub joint: String,
    regs: Registers,
    /// What it saved into flash: what it starts with when powered on.
    flash: Registers,
    /// Whether it has the voltage to run.
    on: bool,
    status: u8,
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
    /// Its drive's and its coil's temperatures, °C, which it reports.
    drive_c: f64,
    coil_c: f64,
}

impl Motor {
    pub fn new(joint: String, regs: Registers) -> Self {
        Self {
            joint,
            flash: regs.clone(),
            regs,
            on: true,
            status: status::DISABLED,
            command: Command::None,
            integral: 0.0,
            last_law: None,
            heard: 0.0,
            zero: 0.0,
            torque: 0.0,
            drive_c: crate::thermal::AMBIENT,
            coil_c: crate::thermal::AMBIENT,
        }
    }

    pub fn registers(&self) -> &Registers {
        &self.regs
    }

    /// Its CAN id, where its commands go.
    pub fn id(&self) -> u16 {
        self.regs.id
    }

    pub fn status(&self) -> u8 {
        self.status
    }

    /// Whether it has the voltage to run: otherwise it neither hears nor
    /// answers.
    pub fn on(&self) -> bool {
        self.on
    }

    pub fn torque(&self) -> f64 {
        self.torque
    }

    /// The mode it's in.
    pub fn mode(&self) -> u32 {
        self.regs.mode
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
            None if self.regs.mode != mode::MIT => return None,
            None => {
                if self.status == status::ENABLED {
                    self.command = Command::Mit(Mit::decode(data, self.regs.limits));
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
        if mode != self.regs.mode {
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

    /// A register request for this motor. Reads and writes of a register
    /// in Damiao's table are answered with its value after them, so a write
    /// it refused answers with the value it kept; registers whose value
    /// drydock doesn't know, and registers outside the table, get no
    /// answer. A write takes effect at once, so a new id or master id
    /// answers as the new one (whether the real motors answer as the old
    /// one isn't in the sources). Refresh answers with feedback; save, when
    /// disabled, with done.
    pub fn on_register(&mut self, request: Register, now: f64, q: f64, v: f64) -> Option<Frame> {
        self.heard = now;
        let (rid, write) = match request {
            Register::Refresh { .. } => return Some(self.feedback(q, v)),
            Register::Save { .. } => return self.save(),
            Register::Read { rid, .. } => (rid, false),
            Register::Write { rid, value, .. } => {
                self.write(rid, value);
                (rid, true)
            }
        };
        let value = match rid {
            reg::XOUT => ((q - self.zero) as f32).to_le_bytes(),
            _ => self.regs.get(rid)?,
        };
        Some(Frame::new(self.regs.master_id, RegisterReply { motor: self.regs.id, write, rid, value }.encode()))
    }

    /// Its registers into flash, where it starts from when powered on,
    /// which a motor does only while disabled (Damiao's manual). What an
    /// enabled one answers isn't in the sources: nothing, here.
    fn save(&mut self) -> Option<Frame> {
        if self.status == status::ENABLED {
            return None;
        }
        self.flash = self.regs.clone();
        Some(Frame::new(self.regs.master_id, Saved { motor: self.regs.id }.encode()))
    }

    /// A write within the register's range (Damiao's table) takes; others
    /// don't, nor do writes to read-only registers. A new mode starts from
    /// no command.
    fn write(&mut self, rid: u8, value: [u8; 4]) {
        if !reg::takes(rid, value) {
            return;
        }
        if rid == reg::CTRL_MODE && u32::from_le_bytes(value) != self.regs.mode {
            self.reset();
        }
        self.regs.set(rid, value);
    }

    /// The torque it asks for until the next step, while enabled: the MIT
    /// law on an MIT command; in the other modes, its own speed loop, and a
    /// position loop around it for position-velocity and position-force.
    /// None otherwise. A motor past its timeout disables itself here.
    pub fn law(&mut self, now: f64, q: f64, v: f64) -> f64 {
        let dt = self.last_law.map_or(0.0, |t| (now - t).max(0.0));
        self.last_law = Some(now);
        if self.status == status::ENABLED && self.regs.timeout().is_some_and(|t| now - self.heard > t) {
            self.status = status::COMMUNICATION_LOST;
        }
        if self.status != status::ENABLED {
            return 0.0;
        }
        let p = q - self.zero;
        let tmax = self.regs.limits.t as f64;
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
        let gain = SPEED_GAIN * self.regs.limits.t as f64;
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
        let limit = self.regs.over_temperature as f64;
        if drive >= limit {
            self.trip(status::MOS_OVERTEMPERATURE);
        } else if coil >= limit {
            self.trip(status::COIL_OVERTEMPERATURE);
        }
    }

    /// The bus's voltage at the motor. Below [`POWER_ON`], it's off; back
    /// above, it starts as at power-on: disabled, from the registers it last
    /// saved. Whether it keeps its zero isn't in the sources: it does, here.
    /// Enabled, below `UV_Value` or above `OV_Value`, it stops with an
    /// undervoltage or overvoltage error, until cleared, as it does for
    /// heat.
    pub fn power(&mut self, volts: f64) {
        if volts < POWER_ON {
            self.on = false;
            self.status = status::DISABLED;
            self.torque = 0.0;
            return;
        }
        if !self.on {
            self.on = true;
            self.regs = self.flash.clone();
            self.reset();
        }
        if volts < self.regs.under_voltage as f64 {
            self.trip(status::UNDERVOLTAGE);
        } else if volts > self.regs.over_voltage as f64 {
            self.trip(status::OVERVOLTAGE);
        }
    }

    /// Stops it with an error, if it's enabled.
    pub fn trip(&mut self, error: u8) {
        if self.status == status::ENABLED {
            self.status = error;
        }
    }

    fn feedback(&self, q: f64, v: f64) -> Frame {
        let f = Feedback {
            id: (self.regs.id & 0xF) as u8,
            status: self.status,
            p: (q - self.zero) as f32,
            v: v as f32,
            t: self.torque as f32,
            t_mos: self.drive_c.round().clamp(0.0, 255.0) as u8,
            t_rotor: self.coil_c.round().clamp(0.0, 255.0) as u8,
        };
        Frame::new(self.regs.master_id, f.encode(self.regs.limits))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::damiao::Model;

    fn motor() -> Motor {
        let mut regs = Registers::new(1, 0x11, Model::DM4310.limits(), 24.0);
        regs.timeout = 2000;
        Motor::new("j".into(), regs)
    }

    fn command(m: &Motor, c: Mit) -> [u8; 8] {
        c.encode(m.regs.limits)
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
        let d = Mit::decode(&c, m.regs.limits);
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
        assert!((Feedback::decode(&f.eight().unwrap(), m.regs.limits).t + 1.5).abs() < 0.01);
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
        assert!((Feedback::decode(&f.eight().unwrap(), m.regs.limits).p - 0.25).abs() < 1e-3);
    }

    #[test]
    fn off_it_neither_runs_nor_remembers_what_it_didnt_save() {
        let mut m = motor();
        let write = |m: &mut Motor, rid, x: f32| {
            m.on_register(Register::Write { motor: 1, rid, value: x.to_le_bytes() }, 0.0, 0.0, 0.0);
        };
        write(&mut m, reg::VMAX, 20.0);
        assert!(m.on_register(Register::Save { motor: 1 }, 0.0, 0.0, 0.0).is_some());
        write(&mut m, reg::PMAX, 5.0);
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.power(5.0);
        assert!(!m.on() && m.status() == status::DISABLED);
        m.power(24.0);
        assert!(m.on() && m.status() == status::DISABLED);
        assert_eq!((m.regs.limits.v, m.regs.limits.p), (20.0, 12.5), "saved, and not");
    }

    #[test]
    fn enabled_out_of_its_voltages_it_stops() {
        let mut m = motor();
        m.power(12.0);
        assert_eq!(m.status(), status::DISABLED, "disabled, nothing to stop");
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.power(12.0);
        assert_eq!(m.status(), status::UNDERVOLTAGE);
        m.on_frame(&Special::ClearError.encode(), 0.0, 0.0, 0.0);
        m.on_frame(&Special::Enable.encode(), 0.0, 0.0, 0.0);
        m.power(40.0);
        assert_eq!(m.status(), status::OVERVOLTAGE);
    }

    #[test]
    fn registers() {
        let mut m = motor();
        let read = |m: &mut Motor, rid| m.on_register(Register::Read { motor: 1, rid }, 0.0, 0.0, 0.0);
        let f = read(&mut m, reg::PMAX).unwrap();
        assert_eq!(f.id, 0x11);
        assert_eq!(RegisterReply::decode(&f.eight().unwrap()).unwrap().value, 12.5f32.to_le_bytes());
        assert_eq!(
            RegisterReply::decode(&read(&mut m, reg::TIMEOUT).unwrap().eight().unwrap()).unwrap().value,
            2000u32.to_le_bytes()
        );
        assert!(read(&mut m, 37).is_none(), "not in Damiao's table");
        assert!(read(&mut m, 25).is_none(), "KP_ASR: its factory value isn't published");
        let uv = RegisterReply::decode(&read(&mut m, reg::UV_VALUE).unwrap().eight().unwrap()).unwrap();
        assert_eq!(uv.value, 15.0f32.to_le_bytes());

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
        assert_eq!(m.regs.limits.t, 20.0);
    }
}
