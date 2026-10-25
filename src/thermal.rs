//! How hot a motor gets: its coil, and its drive (the transistors that
//! switch the coil's current), each a temperature that heats with the
//! torque the motor gives and cools towards the air around it.
//!
//! Heat goes as the current squared, and the current as the torque, so each
//! temperature heads for `ambient + rise × (torque / rated torque)²`, as fast
//! as its time constant says. A motor held at its rated torque (what it may
//! give for as long as it likes) settles below its over-temperature
//! threshold; held at its peak, it reaches it, the drive first.
//!
//! Damiao's datasheets give a motor's rated and peak torque, not how fast or
//! how far it heats, so the rises and time constants here are assumptions,
//! chosen to agree with what they do give: rated torque settles 10 °C under
//! the default threshold, and peak torque trips a motor within seconds.
//! Whether the motor trips, and at what temperature, is its firmware's
//! (`OT_Value`, `damiao::motor`).

/// The air around the motors, °C.
pub const AMBIENT: f64 = 25.0;

/// One temperature: how far above ambient it settles at rated torque, °C,
/// and its time constant, seconds.
#[derive(Debug, Clone, Copy)]
struct Part {
    rise: f64,
    time_constant: f64,
}

/// The coil: 110 °C at rated torque, forever; slow, as copper and its
/// housing are heavy. Assumed.
const COIL: Part = Part { rise: 85.0, time_constant: 120.0 };
/// The drive: cooler at rated torque, but quick: a few grams of silicon on
/// a board. Assumed.
const DRIVE: Part = Part { rise: 45.0, time_constant: 20.0 };

/// How much faster a motor heats when it has lost its cooling: the
/// `overheat` fault.
pub const LOST_COOLING: f64 = 5.0;

#[derive(Debug, Clone, Copy)]
pub struct Thermal {
    /// N m, at the output, as the torques around it.
    pub rated_torque: f64,
    /// °C.
    pub coil: f64,
    pub drive: f64,
}

impl Thermal {
    pub fn new(rated_torque: f64) -> Self {
        Self { rated_torque, coil: AMBIENT, drive: AMBIENT }
    }

    /// `dt` seconds giving `torque`, its heating `heating` times what it
    /// would be (1, or [`LOST_COOLING`]).
    pub fn step(&mut self, dt: f64, torque: f64, heating: f64) {
        let load = (torque / self.rated_torque).powi(2) * heating;
        let toward = |t: f64, part: Part| {
            let target = AMBIENT + part.rise * load;
            t + (target - t) * (1.0 - (-dt / part.time_constant).exp())
        };
        self.coil = toward(self.coil, COIL);
        self.drive = toward(self.drive, DRIVE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hold(t: &mut Thermal, seconds: f64, torque: f64, heating: f64) {
        for _ in 0..(seconds * 100.0) as usize {
            t.step(0.01, torque, heating);
        }
    }

    #[test]
    fn rated_torque_settles_under_the_default_threshold() {
        let mut t = Thermal::new(3.0);
        hold(&mut t, 3600.0, 3.0, 1.0);
        assert!((t.coil - 110.0).abs() < 0.1 && (t.drive - 70.0).abs() < 0.1, "{t:?}");
    }

    #[test]
    fn peak_torque_heats_the_drive_past_it_within_seconds() {
        let mut t = Thermal::new(3.0);
        hold(&mut t, 4.0, 9.0, 1.0);
        assert!(t.drive < 120.0, "{t:?}");
        hold(&mut t, 2.0, 9.0, 1.0);
        assert!(t.drive > 120.0 && t.coil < 120.0, "{t:?}");
    }

    #[test]
    fn it_cools_back_down() {
        let mut t = Thermal::new(3.0);
        hold(&mut t, 30.0, 9.0, 1.0);
        hold(&mut t, 600.0, 0.0, 1.0);
        assert!(t.drive < 26.0 && t.coil < 40.0, "{t:?}");
    }

    #[test]
    fn without_its_cooling_rated_torque_overheats_it() {
        let mut t = Thermal::new(3.0);
        hold(&mut t, 60.0, 3.0, LOST_COOLING);
        assert!(t.drive > 120.0, "{t:?}");
    }
}
