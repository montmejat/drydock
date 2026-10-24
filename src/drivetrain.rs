//! Between a motor's firmware and its joint, the physical part: what limits
//! the torque the motor gives (the actuator), how the motor's turning moves
//! its joint (the transmission), what its encoder reads of it, and how it
//! breaks (`effects`). A motor asks for a torque; this says what reaches the
//! joint, and what the motor senses of it.

use serde::Deserialize;

use crate::effects::{self, Encoder, Fault};
use crate::world::Joint;

/// What a motor can physically give.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Actuator {
    /// N m, either way.
    pub peak_torque: f64,
}

impl Actuator {
    /// The torque a motor gives when its law asks for `asked`.
    pub fn torque(&self, asked: f64) -> f64 {
        asked.clamp(-self.peak_torque, self.peak_torque)
    }
}

/// A linear transmission: the joint is at `offset + ratio θ` when the
/// motor is at `θ` (rad), so a motor torque `τ` puts `τ / ratio` on the
/// joint. A slide joint's ratio is in metres per radian.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Transmission {
    pub ratio: f64,
    #[serde(default)]
    pub offset: f64,
}

impl Default for Transmission {
    fn default() -> Self {
        Self { ratio: 1.0, offset: 0.0 }
    }
}

impl Transmission {
    /// The motor's angle and velocity, from its joint's.
    pub fn to_motor(&self, q: f64, v: f64) -> (f64, f64) {
        ((q - self.offset) / self.ratio, v / self.ratio)
    }

    /// The force or torque on the joint, from the motor's torque.
    pub fn to_joint(&self, torque: f64) -> f64 {
        torque / self.ratio
    }
}

/// A motor's way to its joint, and back.
#[derive(Debug, Clone)]
pub struct Drivetrain {
    pub joint: Joint,
    pub actuator: Actuator,
    pub transmission: Transmission,
    pub encoder: Encoder,
    pub fault: Option<Fault>,
}

impl Drivetrain {
    /// What the motor senses at `now`, its joint being at `q`, moving at
    /// `v`: through the transmission, then its encoder.
    pub fn sense(&mut self, now: f64, q: f64, v: f64) -> (f64, f64) {
        let (theta, omega) = self.transmission.to_motor(q, v);
        self.encoder.sense(now, theta, omega, Fault::at(self.fault, now))
    }

    /// The torque the motor gives at `now`, its firmware asking for `asked`:
    /// within its peak, unless it has broken.
    pub fn torque(&self, now: f64, asked: f64) -> f64 {
        effects::torque(self.actuator.torque(asked), Fault::at(self.fault, now))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn torque_stays_within_the_peak() {
        let a = Actuator { peak_torque: 7.0 };
        assert_eq!((a.torque(12.0), a.torque(-12.0), a.torque(3.0)), (7.0, -7.0, 3.0));
    }

    #[test]
    fn a_transmission_trades_travel_for_force() {
        let t = Transmission { ratio: 0.04, offset: 0.044 };
        let (theta, omega) = t.to_motor(0.004, 0.02);
        assert!((theta + 1.0).abs() < 1e-12 && (omega - 0.5).abs() < 1e-12);
        assert!((t.to_joint(2.0) - 50.0).abs() < 1e-9);
    }
}
