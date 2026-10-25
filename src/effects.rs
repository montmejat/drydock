//! What makes a simulated motor behave like a real one, beyond its firmware:
//! an encoder that reads its angle imperfectly ([`Encoder`]), set per motor
//! in the robot file, and the ways a motor breaks ([`Breakdown`]), which a
//! scenario says when (`scenario.rs`).
//!
//! The randomness is seeded: a lockstep run reads the same every time.
//!
//! From keel's first simulation (`archive/mujoco-sim`), where they made a
//! robot's sensors imperfect for its health checks to find.

use std::collections::VecDeque;

use serde::Deserialize;

/// A small, fast, seeded generator (xorshift64*): noise, not cryptography.
#[derive(Debug, Clone)]
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Rng {
        Rng(seed.wrapping_mul(0x9e3779b97f4a7c15) | 1)
    }

    pub fn next_u64(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545f4914f6cdd1d)
    }

    /// Uniform in [0, 1).
    pub fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Normal, mean 0, standard deviation 1 (Box-Muller).
    pub fn normal(&mut self) -> f64 {
        let (u, v) = (self.unit().max(1e-300), self.unit());
        (-2.0 * u.ln()).sqrt() * (std::f64::consts::TAU * v).cos()
    }
}

/// How a motor breaks.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Breakdown {
    /// It gives no torque at all, whatever its firmware asks.
    Dead,
    /// It gives a fifth of what its firmware asks.
    Weak,
    /// Its encoder repeats its last reading: the firmware's loop, closed on
    /// it, goes on holding a joint that has moved.
    Freeze,
    /// Its encoder's noise grows twentyfold.
    Noisy,
    /// It has lost most of its cooling (a blocked heatsink, a dead fan): it
    /// heats five times as fast, for the same torque.
    Overheat,
    /// Its connector is loose: it neither hears frames nor answers. Its
    /// firmware goes on with the last command it heard, until its timeout,
    /// if it has one.
    Disconnected,
    /// Its link loses frames: about one in ten, each way.
    Lossy,
    /// A phase shorted: as soon as it drives, its current runs away, and
    /// its firmware stops it with an over-current error.
    Shorted,
}

impl Breakdown {
    pub fn name(self) -> &'static str {
        match self {
            Breakdown::Dead => "dead",
            Breakdown::Weak => "weak",
            Breakdown::Freeze => "freeze",
            Breakdown::Noisy => "noisy",
            Breakdown::Overheat => "overheat",
            Breakdown::Disconnected => "disconnected",
            Breakdown::Lossy => "lossy",
            Breakdown::Shorted => "shorted",
        }
    }
}

/// An encoder's imperfections, in the order a reading meets them. All
/// zero by default: a perfect encoder.
#[derive(Debug, Clone, Copy, Default, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct EncoderSettings {
    /// How late a reading is, ms: it shows the joint as it was.
    pub delay_ms: f64,
    /// The angle's offset, drawn once: its standard deviation, rad.
    pub bias: f64,
    /// How fast the offset wanders: a random walk, rad per √second.
    pub drift: f64,
    /// White noise on the angle: its standard deviation, rad.
    pub noise: f64,
    /// The smallest step it tells apart, rad (2π / counts per turn); 0 for
    /// none.
    pub quantum: f64,
}

/// What a motor's firmware senses of its own angle and velocity.
#[derive(Debug, Clone)]
pub struct Encoder {
    settings: EncoderSettings,
    rng: Rng,
    /// True angles and velocities, kept as long as the delay needs.
    history: VecDeque<(f64, f64, f64)>,
    offset: f64,
    last_t: Option<f64>,
    last: (f64, f64),
}

impl Encoder {
    pub fn new(settings: EncoderSettings, seed: u64) -> Encoder {
        let mut rng = Rng::new(seed);
        let offset = settings.bias * rng.normal();
        Encoder { settings, rng, history: VecDeque::new(), offset, last_t: None, last: (0.0, 0.0) }
    }

    /// The reading at `now` (simulated seconds) of a motor truly at `theta`,
    /// turning at `omega`; `broken` says which faults it has.
    pub fn sense(&mut self, now: f64, theta: f64, omega: f64, broken: impl Fn(Breakdown) -> bool) -> (f64, f64) {
        let s = self.settings;
        if broken(Breakdown::Freeze) && self.last_t.is_some() {
            return (self.last.0, 0.0);
        }
        // The newest truth at least `delay` old; the oldest there is, until then.
        self.history.push_back((now, theta, omega));
        let seen = now - s.delay_ms / 1e3;
        while self.history.len() > 1 && self.history[1].0 <= seen {
            self.history.pop_front();
        }
        let (_, theta, omega) = self.history[0];
        let dt = self.last_t.map_or(0.0, |t| (now - t).max(0.0));
        self.last_t = Some(now);
        if s.drift > 0.0 {
            self.offset += s.drift * dt.sqrt() * self.rng.normal();
        }
        let noise = if broken(Breakdown::Noisy) { s.noise.max(1e-3) * 20.0 } else { s.noise };
        let mut reading = theta + self.offset;
        if noise > 0.0 {
            reading += noise * self.rng.normal();
        }
        if s.quantum > 0.0 {
            reading = (reading / s.quantum).round() * s.quantum;
        }
        self.last = (reading, omega);
        self.last
    }
}

/// The torque a motor gives, its firmware asking for `asked`; `broken` says
/// which faults it has.
pub fn torque(asked: f64, broken: impl Fn(Breakdown) -> bool) -> f64 {
    if broken(Breakdown::Dead) {
        0.0
    } else if broken(Breakdown::Weak) {
        asked * 0.2
    } else {
        asked
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_noise_repeats_with_its_seed() {
        let draw = |seed| (0..5).map(|_| Rng::new(seed).normal()).collect::<Vec<_>>();
        assert_eq!(draw(3), draw(3));
        let (mut a, mut b) = (Rng::new(1), Rng::new(2));
        assert_ne!(a.next_u64(), b.next_u64());
    }

    #[test]
    fn noise_has_its_standard_deviation() {
        let mut rng = Rng::new(7);
        let n = 20_000;
        let xs: Vec<f64> = (0..n).map(|_| 0.5 * rng.normal()).collect();
        let mean = xs.iter().sum::<f64>() / n as f64;
        let sd = (xs.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n as f64).sqrt();
        assert!(mean.abs() < 0.02 && (sd - 0.5).abs() < 0.02, "{mean} {sd}");
    }

    #[test]
    fn a_perfect_encoder_reads_the_truth() {
        let mut e = Encoder::new(EncoderSettings::default(), 1);
        assert_eq!(e.sense(0.0, 1.25, -0.5, |_| false), (1.25, -0.5));
    }

    #[test]
    fn an_encoder_counts_in_steps_and_late() {
        let settings = EncoderSettings { quantum: 0.01, delay_ms: 2.0, ..Default::default() };
        let mut e = Encoder::new(settings, 1);
        assert_eq!(e.sense(0.0, 0.123, 0.0, |_| false).0, 0.12);
        assert_eq!(e.sense(0.001, 0.5, 0.0, |_| false).0, 0.12, "still the reading of t = 0");
        assert_eq!(e.sense(0.002, 0.5, 0.0, |_| false).0, 0.12, "the truth of t = 0, 2 ms ago");
        assert_eq!(e.sense(0.003, 0.5, 0.0, |_| false).0, 0.5, "the truth of t = 1 ms");
    }

    #[test]
    fn a_frozen_encoder_repeats_itself() {
        let mut e = Encoder::new(EncoderSettings::default(), 1);
        e.sense(0.0, 0.3, 1.0, |_| false);
        assert_eq!(e.sense(1.0, 0.9, 1.0, |f| f == Breakdown::Freeze), (0.3, 0.0));
    }

    #[test]
    fn a_dead_motor_gives_nothing_and_a_weak_one_a_fifth() {
        assert_eq!(torque(3.0, |f| f == Breakdown::Dead), 0.0);
        assert!((torque(3.0, |f| f == Breakdown::Weak) - 0.6).abs() < 1e-12);
        assert_eq!(torque(3.0, |f| f == Breakdown::Freeze), 3.0);
        // Faults add up: dead and disconnected is still dead.
        assert_eq!(torque(3.0, |f| matches!(f, Breakdown::Dead | Breakdown::Disconnected)), 0.0);
    }
}
