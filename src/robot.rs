//! A robot file: the scene, the bus, and the motors on the bus. The scene
//! (MJCF) says how the robot moves; this file says what drives it.
//!
//! ```yaml
//! scene: arm.xml          # relative to this file
//! keyframe: home          # optional: where it starts
//! bus: vcan0              # optional, vcan0 by default
//! motors:
//!   - { joint: shoulder, model: DM4340, id: 0x01, master_id: 0x11 }
//!   - { joint: elbow, model: DM4310, id: 0x02, master_id: 0x12, peak_torque: 7, timeout_ms: 100 }
//! ```

use std::path::{Path, PathBuf};

use serde::Deserialize;

use crate::damiao::{Model, REGISTER_ID};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Robot {
    pub scene: PathBuf,
    #[serde(default)]
    pub keyframe: Option<String>,
    #[serde(default = "default_bus")]
    pub bus: String,
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
        }
        Ok(())
    }
}
