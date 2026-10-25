//! A robot file: the scene, and the buses that drive it, each with its
//! motors. The scene (MJCF) says how the robot moves; this file says what
//! drives it.
//!
//! ```yaml
//! scene: arm.xml              # relative to this file; or a robot's published model, fetched once:
//! # scene: { git: https://github.com/enactic/openarm_mujoco, commit: a3c8a57…, file: v1/openarm.xml }
//! keyframe: home              # optional: where it starts
//! buses:
//!   - can: vcan0              # the interface
//!     fd: false               # optional: whether the motors are on CAN FD
//!     bitrate: 1000000        # optional: bit/s, 1 Mbit/s by default
//!     data_bitrate: 5000000   # optional: CAN FD's data phase, 5 Mbit/s by default
//!     reply_us: 0             # optional: how long a motor takes to answer
//!     motors:
//!       - { joint: shoulder, model: DM4340, id: 0x01, master_id: 0x11 }
//!       - { joint: elbow, model: DM4310, id: 0x02, master_id: 0x12, peak_torque: 7, timeout_ms: 100 }
//!       # A gripper: the motor's angle moves a finger along a slide.
//!       - { joint: finger, model: DM4310, id: 0x08, master_id: 0x18, transmission: { ratio: 0.042, offset: 0.044 } }
//!       # An imperfect encoder. When motors break is a scenario's (`scenario.rs`).
//!       - { joint: wrist, model: DM4310, id: 0x03, master_id: 0x13, encoder: { noise: 0.0005, quantum: 0.0004 } }
//! seed: 0                     # optional: for the encoders' noise
//! ```

use std::fmt;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde::Deserialize;

use crate::can::timing::Rates;
use crate::damiao::{Limits, Model, REGISTER_ID};
use crate::drivetrain::Transmission;
use crate::effects::EncoderSettings;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Robot {
    pub scene: Scene,
    #[serde(default)]
    pub keyframe: Option<String>,
    pub buses: Vec<BusSpec>,
    /// For the encoders' noise: each motor draws its own from it.
    #[serde(default)]
    pub seed: u64,
    /// The directory the file is in, which paths in it are relative to.
    #[serde(skip)]
    pub dir: PathBuf,
}

/// Where the robot's model is: a file, or a file in a repository, at a
/// pinned commit, fetched once into drydock's cache.
#[derive(Debug, Clone, Deserialize)]
#[serde(untagged)]
pub enum Scene {
    File(PathBuf),
    Git { git: String, commit: String, file: PathBuf },
}

impl fmt::Display for Scene {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        match self {
            Scene::File(path) => write!(f, "{}", path.display()),
            Scene::Git { git, commit, file } => {
                write!(f, "{} at {}: {}", repository_name(git), &commit[..commit.len().min(7)], file.display())
            }
        }
    }
}

fn repository_name(git: &str) -> &str {
    git.trim_end_matches('/').trim_end_matches(".git").rsplit('/').next().unwrap_or(git)
}

/// Where drydock keeps what it fetches: `$DRYDOCK_CACHE`, or
/// `~/.cache/drydock`.
pub fn cache_dir() -> PathBuf {
    match std::env::var_os("DRYDOCK_CACHE") {
        Some(dir) => PathBuf::from(dir),
        None => PathBuf::from(std::env::var_os("HOME").unwrap_or_default()).join(".cache/drydock"),
    }
}

/// A repository's commit, fetched into the cache unless it's there already:
/// one commit deep, only the top folder `file` is in (robot repositories
/// carry many models' meshes), with `git`, into a folder renamed into place
/// once whole.
fn fetch(git: &str, commit: &str, file: &Path) -> Result<PathBuf, String> {
    let short = &commit[..commit.len().min(12)];
    let dir = cache_dir().join("models").join(format!("{}-{short}", repository_name(git)));
    if dir.is_dir() {
        return Ok(dir);
    }
    eprintln!("drydock: fetching {} at {short}, once, into {}", repository_name(git), dir.display());
    let part = dir.with_extension("part");
    let _ = std::fs::remove_dir_all(&part);
    std::fs::create_dir_all(&part).map_err(|e| format!("can't make {}: {e}", part.display()))?;
    let folder = file.components().next().filter(|_| file.components().count() > 1);
    let folder = folder.map(|c| c.as_os_str().to_string_lossy().into_owned());
    let mut steps: Vec<Vec<&str>> = vec![vec!["init", "-q"]];
    if let Some(folder) = &folder {
        steps.push(vec!["sparse-checkout", "set", folder]);
    }
    steps.push(vec!["fetch", "--depth", "1", "--filter=blob:none", git, commit]);
    steps.push(vec!["-c", "advice.detachedHead=false", "checkout", "-q", "FETCH_HEAD"]);
    for args in &steps {
        let status = Command::new("git").arg("-C").arg(&part).args(args).status();
        match status {
            Ok(s) if s.success() => {}
            Ok(_) => return Err(format!("couldn't fetch {git} at {commit} (`git {}` failed)", args.join(" "))),
            Err(e) => return Err(format!("can't run git, which fetches robot models: {e}")),
        }
    }
    std::fs::rename(&part, &dir).map_err(|e| format!("can't move {} into place: {e}", part.display()))?;
    Ok(dir)
}

/// A CAN bus of Damiao motors.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BusSpec {
    /// The interface: `vcan0`.
    pub can: String,
    /// Whether the motors are set to CAN FD. Classic CAN by default.
    #[serde(default)]
    pub fd: bool,
    #[serde(default = "default_bitrate")]
    pub bitrate: f64,
    #[serde(default = "default_data_bitrate")]
    pub data_bitrate: f64,
    /// How long a motor takes from a frame to its answer, µs. Unknown for
    /// Damiao's motors: none by default.
    #[serde(default)]
    pub reply_us: f64,
    pub motors: Vec<MotorSpec>,
}

impl BusSpec {
    pub fn rates(&self) -> Rates {
        Rates { nominal: self.bitrate, data: self.data_bitrate }
    }
}

fn default_bitrate() -> f64 {
    1e6
}

fn default_data_bitrate() -> f64 {
    5e6
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
    /// What its encoder gets wrong: perfect by default.
    #[serde(default)]
    pub encoder: EncoderSettings,
}

#[derive(Debug, Clone, Copy, Default, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LimitsSpec {
    pub p: Option<f32>,
    pub v: Option<f32>,
    pub t: Option<f32>,
}

impl MotorSpec {
    pub fn limits(&self) -> Limits {
        let l = self.model.limits();
        Limits { p: self.limits.p.unwrap_or(l.p), v: self.limits.v.unwrap_or(l.v), t: self.limits.t.unwrap_or(l.t) }
    }

    pub fn peak_torque(&self) -> f64 {
        self.peak_torque.unwrap_or(self.limits().t as f64)
    }
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

    /// The scene's file, fetching it first if it comes from a repository
    /// that isn't in the cache yet.
    pub fn scene_path(&self) -> Result<PathBuf, String> {
        match &self.scene {
            Scene::File(path) => Ok(self.dir.join(path)),
            Scene::Git { git, commit, file } => Ok(fetch(git, commit, file)?.join(file)),
        }
    }

    /// Every motor, with its bus.
    pub fn motors(&self) -> impl Iterator<Item = (&BusSpec, &MotorSpec)> {
        self.buses.iter().flat_map(|b| b.motors.iter().map(move |m| (b, m)))
    }

    fn check(&self) -> Result<(), String> {
        if self.buses.is_empty() {
            return Err("no buses".into());
        }
        let mut interfaces = Vec::new();
        for bus in &self.buses {
            if interfaces.contains(&&bus.can) {
                return Err(format!("{} is listed twice", bus.can));
            }
            interfaces.push(&bus.can);
            check_bus(bus).map_err(|e| format!("{}: {e}", bus.can))?;
        }
        let mut joints: Vec<&str> = self.motors().map(|(_, m)| m.joint.as_str()).collect();
        joints.sort();
        if let Some(w) = joints.windows(2).find(|w| w[0] == w[1]) {
            return Err(format!("two motors drive {}", w[0]));
        }
        Ok(())
    }
}

/// Ids must be 11-bit, apart from each other and from `0x7FF` (on one bus:
/// two buses may use the same ids, as two arms do).
fn check_bus(bus: &BusSpec) -> Result<(), String> {
    if bus.motors.is_empty() {
        return Err("no motors".into());
    }
    if [bus.bitrate, bus.data_bitrate].iter().any(|r| !(r.is_finite() && *r > 0.0))
        || !(bus.reply_us.is_finite() && bus.reply_us >= 0.0)
    {
        return Err("bit rates are above zero, reply_us zero or more".into());
    }
    let mut ids: Vec<(u16, &str)> = Vec::new();
    for m in &bus.motors {
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
        let e = m.encoder;
        if [e.delay_ms, e.bias, e.drift, e.noise, e.quantum].iter().any(|x| !(x.is_finite() && *x >= 0.0)) {
            return Err(format!("{}: an encoder's settings are zero or more", m.joint));
        }
    }
    Ok(())
}
