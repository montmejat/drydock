//! A robot file: the scene, the bus, and the motors on the bus. The scene
//! (MJCF) says how the robot moves; this file says what drives it.
//!
//! ```yaml
//! scene: arm.xml          # relative to this file
//! keyframe: home          # optional: where it starts
//! bus: vcan0              # optional, vcan0 by default
//! can_fd: false           # optional: whether the motors are on CAN FD
//! motors:
//!   - { joint: shoulder, model: DM4340, id: 0x01, master_id: 0x11 }
//!   - { joint: elbow, model: DM4310, id: 0x02, master_id: 0x12, peak_torque: 7, timeout_ms: 100 }
//!   # A gripper: the motor's angle moves a finger along a slide.
//!   - { joint: finger, model: DM4310, id: 0x08, master_id: 0x18, transmission: { ratio: 0.042, offset: 0.044 } }
//! ```

use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::damiao::{Limits, Model, REGISTER_ID};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Robot {
    pub scene: PathBuf,
    #[serde(default)]
    pub keyframe: Option<String>,
    #[serde(default = "default_bus")]
    pub bus: String,
    /// Whether the motors are set to CAN FD. Classic CAN by default.
    #[serde(default)]
    pub can_fd: bool,
    pub motors: Vec<MotorSpec>,
    /// The directory the file is in, which paths in it are relative to.
    #[serde(skip)]
    pub dir: PathBuf,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MotorSpec {
    pub joint: String,
    pub model: Model,
    pub id: u16,
    pub master_id: u16,
    /// N m: by default, the model's TMAX.
    #[serde(default)]
    pub peak_torque: Option<f64>,
    /// Off by default, as Damiao's motors leave the factory.
    #[serde(default)]
    pub timeout_ms: Option<f64>,
    /// PMAX, VMAX, TMAX, where they differ from the model's: they are
    /// registers, set per motor.
    #[serde(default)]
    pub limits: LimitsSpec,
    /// Between the motor and its joint, when they don't turn together.
    #[serde(default)]
    pub transmission: Transmission,
}

#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LimitsSpec {
    pub p: Option<f32>,
    pub v: Option<f32>,
    pub t: Option<f32>,
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

impl MotorSpec {
    pub fn limits(&self) -> Limits {
        let l = self.model.limits();
        Limits { p: self.limits.p.unwrap_or(l.p), v: self.limits.v.unwrap_or(l.v), t: self.limits.t.unwrap_or(l.t) }
    }
}

fn default_bus() -> String {
    "vcan0".into()
}

impl Robot {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, String> {
        let path = path.as_ref();
        let text = std::fs::read_to_string(path).map_err(|e| format!("can't read {}: {e}", path.display()))?;
        let mut robot: Robot = serde_yaml::from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        robot.dir = path.parent().unwrap_or(Path::new(".")).to_path_buf();
        robot.check().map_err(|e| format!("{}: {e}", path.display()))?;
        Ok(robot)
    }

    pub fn scene_path(&self) -> PathBuf {
        self.dir.join(&self.scene)
    }

    /// Ids must be 11-bit, apart from each other and from `0x7FF`.
    fn check(&self) -> Result<(), String> {
        if self.motors.is_empty() {
            return Err("no motors".into());
        }
        let mut ids: Vec<(u16, &str)> = Vec::new();
        for m in &self.motors {
            for (id, what) in [(m.id, "id"), (m.master_id, "master_id")] {
                if id >= REGISTER_ID {
                    return Err(format!("{}'s {what} {id:#x} isn't below 0x7FF", m.joint));
                }
                if let Some((_, other)) = ids.iter().find(|(i, _)| *i == id) {
                    return Err(format!("{}'s {what} {id:#x} is also {other}'s", m.joint));
                }
                ids.push((id, &m.joint));
            }
            if [m.peak_torque, m.timeout_ms].into_iter().flatten().any(|x| x.is_nan() || x <= 0.0) {
                return Err(format!("{}: peak_torque and timeout_ms are above zero", m.joint));
            }
            if [m.limits.p, m.limits.v, m.limits.t].into_iter().flatten().any(|x| x.is_nan() || x <= 0.0) {
                return Err(format!("{}: limits are above zero", m.joint));
            }
            if !m.transmission.ratio.is_finite() || m.transmission.ratio == 0.0 || !m.transmission.offset.is_finite() {
                return Err(format!("{}: a transmission's ratio isn't zero", m.joint));
            }
        }
        Ok(())
    }
}
