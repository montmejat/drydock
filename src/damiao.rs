//! Damiao's motors in MIT mode, as they speak on a classic CAN bus: the
//! frames, and how numbers are packed into them.
//!
//! From Damiao's manual for the DM-J4310-2EC and their reference driver
//! (`DM_CAN.py` in DM_Control_Python):
//!
//! - **A command** goes to the motor's CAN id: a target position and
//!   velocity, two gains and a feed-forward torque, in 16, 12, 12, 12 and 12
//!   bits. The motor applies `kp (p - p_des) + kd (v - v_des) + t_ff`.
//! - **Four special frames** go to the same id: seven `FF` bytes, then `FC`
//!   to enable, `FD` to disable, `FE` to make the present position zero, or
//!   `FB` to clear an error.
//! - **The motor answers each frame** with feedback on its master id: its
//!   status and id, then position, velocity, torque and two temperatures.
//! - **Registers** are read and written through id `0x7FF`, and answered on
//!   the master id.
//!
//! Positions, velocities and torques are packed into the ranges set by the
//! motor's PMAX, VMAX and TMAX registers; a value outside its range is
//! clamped to it.

use serde::Deserialize;

/// Where register requests go, for every motor on the bus.
pub const REGISTER_ID: u16 = 0x7FF;

/// The gains' ranges, the same on every motor.
pub const KP_MAX: f32 = 500.0;
pub const KD_MAX: f32 = 5.0;

/// The status in a feedback frame's high nibble.
pub mod status {
    pub const DISABLED: u8 = 0x0;
    pub const ENABLED: u8 = 0x1;
    pub const OVERVOLTAGE: u8 = 0x8;
    pub const UNDERVOLTAGE: u8 = 0x9;
    pub const OVERCURRENT: u8 = 0xA;
    pub const MOS_OVERTEMPERATURE: u8 = 0xB;
    pub const COIL_OVERTEMPERATURE: u8 = 0xC;
    pub const COMMUNICATION_LOST: u8 = 0xD;
    pub const OVERLOAD: u8 = 0xE;
}

/// The registers drydock simulates, by number.
pub mod reg {
    pub const MST_ID: u8 = 7;
    pub const ESC_ID: u8 = 8;
    pub const TIMEOUT: u8 = 9;
    pub const CTRL_MODE: u8 = 10;
    pub const PMAX: u8 = 21;
    pub const VMAX: u8 = 22;
    pub const TMAX: u8 = 23;

    /// Whether a register holds an integer (otherwise a float), as the
    /// reference driver decides.
    pub fn is_integer(rid: u8) -> bool {
        matches!(rid, 7..=10 | 13..=16 | 35..=36)
    }

    /// CTRL_MODE's value for MIT mode.
    pub const MIT_MODE: u32 = 1;
}

/// The ranges a motor packs position (rad), velocity (rad/s) and torque
/// (N m) into: its PMAX, VMAX and TMAX registers.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub p: f32,
    pub v: f32,
    pub t: f32,
}

/// Damiao's motors, as the reference driver names them.
#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum Model {
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
}

impl Model {
    /// The ranges a motor leaves the factory with (`Limit_Param`).
    pub fn limits(self) -> Limits {
        let (p, v, t) = match self {
            Model::DM4310 => (12.5, 30.0, 10.0),
            Model::DM4310_48V => (12.5, 50.0, 10.0),
            Model::DM4340 => (12.5, 8.0, 28.0),
            Model::DM4340_48V => (12.5, 10.0, 28.0),
            Model::DM6006 => (12.5, 45.0, 20.0),
            Model::DM8006 => (12.5, 45.0, 40.0),
            Model::DM8009 => (12.5, 45.0, 54.0),
            Model::DM10010L => (12.5, 25.0, 200.0),
            Model::DM10010 => (12.5, 20.0, 200.0),
            Model::DMH3510 => (12.5, 280.0, 1.0),
            Model::DMH6215 => (12.5, 45.0, 10.0),
            Model::DMG6220 => (12.5, 45.0, 10.0),
        };
        Limits { p, v, t }
    }
}

fn to_uint(x: f32, min: f32, max: f32, bits: u32) -> u16 {
    let x = x.clamp(min, max);
    ((x - min) / (max - min) * ((1u32 << bits) - 1) as f32) as u16
}

fn to_float(x: u16, min: f32, max: f32, bits: u32) -> f32 {
    x as f32 / ((1u32 << bits) - 1) as f32 * (max - min) + min
}

/// An MIT-mode command.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Mit {
    pub p: f32,
    pub v: f32,
    pub kp: f32,
    pub kd: f32,
    pub t: f32,
}

impl Mit {
    pub fn encode(&self, l: Limits) -> [u8; 8] {
        let p = to_uint(self.p, -l.p, l.p, 16);
        let v = to_uint(self.v, -l.v, l.v, 12);
        let kp = to_uint(self.kp, 0.0, KP_MAX, 12);
        let kd = to_uint(self.kd, 0.0, KD_MAX, 12);
        let t = to_uint(self.t, -l.t, l.t, 12);
        [
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            ((v & 0xF) << 4) as u8 | (kp >> 8) as u8,
            kp as u8,
            (kd >> 4) as u8,
            ((kd & 0xF) << 4) as u8 | (t >> 8) as u8,
            t as u8,
        ]
    }

    pub fn decode(d: &[u8; 8], l: Limits) -> Self {
        let p = (d[0] as u16) << 8 | d[1] as u16;
        let v = (d[2] as u16) << 4 | (d[3] >> 4) as u16;
        let kp = ((d[3] & 0xF) as u16) << 8 | d[4] as u16;
        let kd = (d[5] as u16) << 4 | (d[6] >> 4) as u16;
        let t = ((d[6] & 0xF) as u16) << 8 | d[7] as u16;
        Mit {
            p: to_float(p, -l.p, l.p, 16),
            v: to_float(v, -l.v, l.v, 12),
            kp: to_float(kp, 0.0, KP_MAX, 12),
            kd: to_float(kd, 0.0, KD_MAX, 12),
            t: to_float(t, -l.t, l.t, 12),
        }
    }
}

/// What a motor says after each frame it gets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Feedback {
    /// The low nibble of the motor's CAN id.
    pub id: u8,
    /// One of [`status`].
    pub status: u8,
    pub p: f32,
    pub v: f32,
    pub t: f32,
    /// The drive's MOSFETs' and the coil's temperatures, °C.
    pub t_mos: u8,
    pub t_rotor: u8,
}

impl Feedback {
    pub fn encode(&self, l: Limits) -> [u8; 8] {
        let p = to_uint(self.p, -l.p, l.p, 16);
        let v = to_uint(self.v, -l.v, l.v, 12);
        let t = to_uint(self.t, -l.t, l.t, 12);
        [
            self.status << 4 | (self.id & 0xF),
            (p >> 8) as u8,
            p as u8,
            (v >> 4) as u8,
            ((v & 0xF) << 4) as u8 | (t >> 8) as u8,
            t as u8,
            self.t_mos,
            self.t_rotor,
        ]
    }

    pub fn decode(d: &[u8; 8], l: Limits) -> Self {
        let p = (d[1] as u16) << 8 | d[2] as u16;
        let v = (d[3] as u16) << 4 | (d[4] >> 4) as u16;
        let t = ((d[4] & 0xF) as u16) << 8 | d[5] as u16;
        Feedback {
            id: d[0] & 0xF,
            status: d[0] >> 4,
            p: to_float(p, -l.p, l.p, 16),
            v: to_float(v, -l.v, l.v, 12),
            t: to_float(t, -l.t, l.t, 12),
            t_mos: d[6],
            t_rotor: d[7],
        }
    }
}

/// The frames that aren't commands: seven `FF` bytes, then which.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Special {
    Enable,
    Disable,
    /// The present position becomes zero.
    Zero,
    ClearError,
}

impl Special {
    fn byte(self) -> u8 {
        match self {
            Special::Enable => 0xFC,
            Special::Disable => 0xFD,
            Special::Zero => 0xFE,
            Special::ClearError => 0xFB,
        }
    }

    pub fn encode(self) -> [u8; 8] {
        let mut d = [0xFF; 8];
        d[7] = self.byte();
        d
    }

    pub fn decode(d: &[u8; 8]) -> Option<Self> {
        if d[..7] != [0xFF; 7] {
            return None;
        }
        [Special::Enable, Special::Disable, Special::Zero, Special::ClearError].into_iter().find(|s| s.byte() == d[7])
    }
}

/// A frame to `0x7FF`, for the motor whose CAN id is `motor`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Register {
    Read {
        motor: u16,
        rid: u8,
    },
    /// `value` is a little-endian `u32` or `f32`, as [`reg::is_integer`] says.
    Write {
        motor: u16,
        rid: u8,
        value: [u8; 4],
    },
    /// Into flash.
    Save {
        motor: u16,
    },
    /// Asks for a feedback frame.
    Refresh {
        motor: u16,
    },
}

const READ: u8 = 0x33;
const WRITE: u8 = 0x55;
const SAVE: u8 = 0xAA;
const REFRESH: u8 = 0xCC;

impl Register {
    pub fn motor(&self) -> u16 {
        match *self {
            Register::Read { motor, .. }
            | Register::Write { motor, .. }
            | Register::Save { motor }
            | Register::Refresh { motor } => motor,
        }
    }

    pub fn encode(&self) -> [u8; 8] {
        let [lo, hi] = self.motor().to_le_bytes();
        match *self {
            Register::Read { rid, .. } => [lo, hi, READ, rid, 0, 0, 0, 0],
            Register::Write { rid, value: [a, b, c, d], .. } => [lo, hi, WRITE, rid, a, b, c, d],
            Register::Save { .. } => [lo, hi, SAVE, 0, 0, 0, 0, 0],
            Register::Refresh { .. } => [lo, hi, REFRESH, 0, 0, 0, 0, 0],
        }
    }

    pub fn decode(d: &[u8; 8]) -> Option<Self> {
        let motor = u16::from_le_bytes([d[0], d[1]]);
        let rid = d[3];
        Some(match d[2] {
            READ => Register::Read { motor, rid },
            WRITE => Register::Write { motor, rid, value: [d[4], d[5], d[6], d[7]] },
            SAVE => Register::Save { motor },
            REFRESH => Register::Refresh { motor },
            _ => return None,
        })
    }
}

/// A motor's answer to a register read or write: the register's value
/// after it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RegisterReply {
    pub motor: u16,
    /// Answering a write, not a read.
    pub write: bool,
    pub rid: u8,
    pub value: [u8; 4],
}

impl RegisterReply {
    pub fn encode(&self) -> [u8; 8] {
        let [lo, hi] = self.motor.to_le_bytes();
        let [a, b, c, d] = self.value;
        [lo, hi, if self.write { WRITE } else { READ }, self.rid, a, b, c, d]
    }

    /// Feedback frames share the master id, so a reply is told apart by its
    /// third byte.
    pub fn decode(d: &[u8; 8]) -> Option<Self> {
        let write = match d[2] {
            READ => false,
            WRITE => true,
            _ => return None,
        };
        Some(RegisterReply {
            motor: u16::from_le_bytes([d[0], d[1]]),
            write,
            rid: d[3],
            value: [d[4], d[5], d[6], d[7]],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const L: Limits = Limits { p: 12.5, v: 30.0, t: 10.0 };

    #[test]
    fn a_zero_command_packs_as_the_reference_driver_packs_it() {
        let d = Mit::default().encode(L);
        assert_eq!(d, [0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF]);
    }

    #[test]
    fn commands_survive_packing_to_a_quantum() {
        let c = Mit { p: 1.234, v: -2.5, kp: 40.0, kd: 1.2, t: 0.75 };
        let back = Mit::decode(&c.encode(L), L);
        assert!((back.p - c.p).abs() < 25.0 / 65535.0);
        assert!((back.v - c.v).abs() < 60.0 / 4095.0);
        assert!((back.kp - c.kp).abs() < 500.0 / 4095.0);
        assert!((back.kd - c.kd).abs() < 5.0 / 4095.0);
        assert!((back.t - c.t).abs() < 20.0 / 4095.0);
    }

    #[test]
    fn values_out_of_range_clamp() {
        let back = Mit::decode(&Mit { p: 100.0, v: -100.0, kp: 900.0, ..Default::default() }.encode(L), L);
        assert_eq!((back.p, back.v, back.kp), (12.5, -30.0, 500.0));
    }

    #[test]
    fn feedback_carries_status_and_id_in_its_first_byte() {
        let f =
            Feedback { id: 0x12, status: status::COMMUNICATION_LOST, p: 0.5, v: 0.0, t: -1.0, t_mos: 30, t_rotor: 31 };
        let d = f.encode(L);
        assert_eq!(d[0], 0xD2);
        let back = Feedback::decode(&d, L);
        assert_eq!((back.id, back.status, back.t_mos, back.t_rotor), (2, 0xD, 30, 31));
        assert!((back.p - 0.5).abs() < 1e-3 && (back.t + 1.0).abs() < 1e-2);
    }

    #[test]
    fn special_frames() {
        assert_eq!(Special::Enable.encode(), [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]);
        assert_eq!(Special::decode(&Special::ClearError.encode()), Some(Special::ClearError));
        assert_eq!(Special::decode(&[0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00]), None);
    }

    #[test]
    fn register_frames() {
        let w = Register::Write { motor: 0x102, rid: reg::PMAX, value: 3.0f32.to_le_bytes() };
        assert_eq!(w.encode()[..4], [0x02, 0x01, 0x55, 21]);
        assert_eq!(Register::decode(&w.encode()), Some(w));
        let r = RegisterReply { motor: 1, write: false, rid: reg::MST_ID, value: 0x11u32.to_le_bytes() };
        assert_eq!(RegisterReply::decode(&r.encode()), Some(r));
    }
}
