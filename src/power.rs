//! The motors' power: a supply, the wires to it, and the capacitors across
//! the bus, which the motors draw from and, braking, give back to.
//!
//! Without a `power` section in the robot file, the supply is ideal: 24 V,
//! whatever the motors draw or give back, as a large battery would be. With
//! one, it's a bench supply. It holds its voltage behind the wires'
//! resistance, so the bus sags as the motors draw. It can't take current
//! back, so what braking motors give charges the capacitors, and the bus
//! rises until something draws it. With `battery: true`, it's a battery:
//! it sags behind its resistance as the motors draw, and takes back what
//! they give.
//!
//! A motor draws what it gives the joint (torque × speed, negative when
//! braking), what its coil loses as heat, and what its electronics need.
//! Damiao doesn't publish its motors' resistance, their idle draw, or their
//! capacitance, so the last three numbers below are assumptions, chosen to
//! be about what small drives of this size have. Whether a motor stops at a
//! voltage is its firmware's (`UV_Value` and `OV_Value`, `damiao::motor`).

use serde::Deserialize;

/// The ideal supply's voltage, V.
pub const IDEAL_VOLTS: f64 = 24.0;

/// What a motor's coil loses as heat at its rated torque, W per N m of
/// that torque: a DM-J4340, rated 9 N m, loses 9 W. It goes as the torque
/// squared. Assumed.
pub const COPPER_LOSS: f64 = 1.0;
/// What a powered motor's electronics draw, W. Assumed.
pub const IDLE: f64 = 1.0;
/// The capacitance each motor puts across the bus, F. Assumed.
pub const CAPACITANCE: f64 = 470e-6;

/// The robot file's `power`: a bench supply, or a battery.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PowerSpec {
    /// What it's set to, or the battery's voltage unloaded, V.
    pub volts: f64,
    /// The wires' resistance, there and back, and the battery's own, Ω.
    #[serde(default)]
    pub ohms: f64,
    /// A battery: it takes back what braking motors give.
    #[serde(default)]
    pub battery: bool,
}

/// What a motor draws, W, giving `torque` (N m) at `speed` (rad/s), rated
/// for `rated` N m, while powered.
pub fn draw(torque: f64, speed: f64, rated: f64) -> f64 {
    IDLE + torque * speed + COPPER_LOSS * torque * torque / rated
}

#[derive(Debug, Clone)]
pub struct Supply {
    /// What it's set to, V.
    pub set: f64,
    /// The wires' resistance, Ω; none for the ideal supply.
    ohms: f64,
    /// The capacitors', F; none for a supply that takes current back (the
    /// ideal one, a battery).
    farads: Option<f64>,
    /// Across the bus, at the motors, V.
    volts: f64,
}

impl Supply {
    pub fn ideal() -> Self {
        Self { set: IDEAL_VOLTS, ohms: 0.0, farads: None, volts: IDEAL_VOLTS }
    }

    /// The robot file's supply, to `motors` motors.
    pub fn new(spec: PowerSpec, motors: usize) -> Self {
        let farads = (!spec.battery).then_some(CAPACITANCE * motors as f64);
        Self { set: spec.volts, ohms: spec.ohms, farads, volts: spec.volts }
    }

    /// The bus's voltage, at the motors.
    pub fn volts(&self) -> f64 {
        self.volts
    }

    /// `dt` seconds of the motors drawing `watts` between them (negative
    /// when they give back more than they draw).
    pub fn step(&mut self, dt: f64, watts: f64) {
        let amps = watts / self.volts.max(1.0);
        let Some(farads) = self.farads else {
            // V = set - R I, solved for I = watts / V: the battery's voltage
            // under that load, either way.
            let disc = self.set * self.set - 4.0 * self.ohms * watts;
            self.volts = if self.ohms == 0.0 || self.set == 0.0 {
                self.set
            } else if disc >= 0.0 {
                (self.set + disc.sqrt()) / 2.0
            } else {
                self.set / 2.0
            };
            return;
        };
        // The capacitors alone: what they give the motors, or take back.
        let alone = self.volts - amps * dt / farads;
        // The supply, through its wires, pulling them towards what it holds
        // under that draw; it only ever pushes current out.
        let target = self.set - self.ohms * amps.max(0.0);
        let tau = self.ohms * farads;
        let pulled = target + (self.volts - target) * (-dt / tau).exp();
        self.volts = alone.max(pulled).max(0.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(s: &mut Supply, seconds: f64, watts: f64) {
        for _ in 0..(seconds / 1e-3) as usize {
            s.step(1e-3, watts);
        }
    }

    #[test]
    fn the_ideal_supply_holds_whatever_the_motors_do() {
        let mut s = Supply::ideal();
        run(&mut s, 1.0, 500.0);
        assert_eq!(s.volts(), 24.0);
        run(&mut s, 1.0, -500.0);
        assert_eq!(s.volts(), 24.0);
    }

    #[test]
    fn a_bench_supply_sags_through_its_wires() {
        let mut s = Supply::new(PowerSpec { volts: 24.0, ohms: 0.5, battery: false }, 2);
        // 240 W through half an ohm: V = 24 - 0.5 × 240 / V, 16.9 V.
        run(&mut s, 0.1, 240.0);
        assert!((s.volts() - 16.9).abs() < 0.05, "{}", s.volts());
        run(&mut s, 0.1, 0.0);
        assert!((s.volts() - 24.0).abs() < 1e-6, "{}", s.volts());
    }

    #[test]
    fn braking_charges_the_capacitors_of_a_bench_supply() {
        // Eight motors' capacitors, 3.76 mF: 2 J back from braking, more
        // than twice the 0.84 J that takes them from 24 V to 32.
        let mut s = Supply::new(PowerSpec { volts: 24.0, ohms: 0.1, battery: false }, 8);
        run(&mut s, 0.1, -20.0);
        assert!(s.volts() > 32.0, "{}", s.volts());
        // Drawing again, they settle back where the supply holds them.
        run(&mut s, 1.0, 20.0);
        assert!((s.volts() - 23.9).abs() < 0.05, "{}", s.volts());
    }

    #[test]
    fn a_battery_sags_as_the_motors_draw_and_takes_back_what_they_give() {
        let mut s = Supply::new(PowerSpec { volts: 7.4, ohms: 0.1, battery: true }, 15);
        s.step(1e-3, 74.0);
        // 74 W through 0.1 Ω: V = 7.4 - 0.1 × 74 / V, 6.21 V.
        assert!((s.volts() - 6.21).abs() < 0.01, "{}", s.volts());
        s.step(1e-3, -14.8);
        assert!(s.volts() > 7.4 && s.volts() < 7.65, "{}", s.volts());
    }

    #[test]
    fn a_supply_switched_off_leaves_the_capacitors_to_the_motors() {
        let mut s = Supply::new(PowerSpec { volts: 24.0, ohms: 0.1, battery: false }, 2);
        s.set = 0.0;
        // Two motors idling, 2 W, drain 0.94 mF from 24 V to 10 V in about
        // a tenth of a second.
        let mut t = 0.0;
        while s.volts() > 10.0 {
            s.step(1e-3, 2.0 * IDLE);
            t += 1e-3;
        }
        assert!((0.05..0.15).contains(&t), "{t} s");
    }
}
