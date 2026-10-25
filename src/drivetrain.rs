//! Between a motor's firmware and its joint, the physical part: what limits
//! the torque the motor gives (the actuator), how the motor's turning moves
//! its joint (the transmission), what its encoder reads of it, how hot it
//! gets (`thermal`), and how it breaks (`effects`). A motor asks for a torque; this says what reaches the
//! joint, and what the motor senses of it.

use serde::Deserialize;

use crate::effects::{self, Breakdown, Encoder};
use crate::thermal::{Thermal, LOST_COOLING};
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
    pub thermal: Thermal,
    /// What's broken, each until when (or for good).
    pub faults: Vec<(Breakdown, Option<f64>)>,
}

impl Drivetrain {
    /// What the motor senses at `now`, its joint being at `q`, moving at
    /// `v`: through the transmission, then its encoder.
    pub fn sense(&mut self, now: f64, q: f64, v: f64) -> (f64, f64) {
        let (theta, omega) = self.transmission.to_motor(q, v);
        let faults = &self.faults;
        self.encoder.sense(now, theta, omega, |b| faults.iter().any(|&(f, _)| f == b))
    }

    /// The torque the motor gives, its firmware asking for `asked`: within
    /// its peak, unless it has broken.
    pub fn torque(&self, asked: f64) -> f64 {
        effects::torque(self.actuator.torque(asked), |b| self.has(b))
    }

    /// `dt` seconds of giving `torque`: how hot it gets.
    pub fn heat(&mut self, dt: f64, torque: f64) {
        let heating = if self.has(Breakdown::Overheat) { LOST_COOLING } else { 1.0 };
        self.thermal.step(dt, torque, heating);
    }

    /// Whether it has that fault now. Faults add up: a dead motor that's
    /// also disconnected is still dead.
    pub fn has(&self, fault: Breakdown) -> bool {
        self.faults.iter().any(|&(f, _)| f == fault)
    }

    /// The faults that are over at `now`, gone.
    pub fn recover(&mut self, now: f64) -> Vec<Breakdown> {
        let (over, on): (Vec<_>, Vec<_>) =
            self.faults.drain(..).partition(|&(_, until)| until.is_some_and(|u| u <= now));
        self.faults = on;
        over.into_iter().map(|(fault, _)| fault).collect()
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
