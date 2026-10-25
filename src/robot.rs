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
//! # Serial buses of Dynamixel servos, each behind a pseudo-terminal drydock
//! # links at `port`, for the robot's software to open as its serial port.
//! serial:
//!   - port: /tmp/microduck-tty
//!     baud: 1000000           # optional: 1 Mbit/s by default
//!     motors:
//!       - { joint: right_knee, model: XL330-M288, id: 13 }
//! power: { volts: 24, ohms: 0.1 }  # optional: a bench supply, through its wires (`power.rs`)
//! seed: 0                     # optional: for the encoders' noise
//! ```

use std::fmt;
use std::path::{Path, PathBuf};
use std::process::Command;

use serde::Deserialize;

use crate::can::timing::Rates;
use crate::damiao::{bit_rate, reg, Limits, Model, REGISTER_ID};
use crate::drivetrain::Transmission;
use crate::effects::EncoderSettings;
use crate::power::PowerSpec;

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Robot {
    pub scene: Scene,
    #[serde(default)]
    pub keyframe: Option<String>,
    /// CAN buses of Damiao motors.
    #[serde(default)]
    pub buses: Vec<BusSpec>,
    /// Serial buses of Dynamixel servos.
    #[serde(default)]
    pub serial: Vec<SerialSpec>,
    /// A bench supply; an ideal one without.
    #[serde(default)]
    pub power: Option<PowerSpec>,
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
    /// What it may give for as long as it likes, N m: what sets how hot it
    /// gets (`thermal.rs`). By default, a third of its peak torque, which
    /// agrees with Damiao's ratings within a tenth where they're published.
    #[serde(default)]
    pub rated_torque: Option<f64>,
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

/// A half-duplex serial bus of Dynamixel servos (Protocol 2.0).
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerialSpec {
    /// Where drydock links its end of the bus: the path the robot's
    /// software opens, as it would `/dev/ttyS2`.
    pub port: PathBuf,
    /// Bits per second, as the servos are set: one of their Baud Rate
    /// register's.
    #[serde(default = "default_bitrate")]
    pub baud: f64,
    pub motors: Vec<ServoSpec>,
    /// An IMU board on the same bus, as the Microduck's.
    #[serde(default)]
    pub imu: Option<ImuSpec>,
}

/// The Microduck's IMU board (`dynamixel::imu`): its id, the site in the
/// scene it senses the motion of, its frame in that site's (w, x, y, z),
/// and its gyro's noise, rad/s.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ImuSpec {
    pub id: u8,
    pub site: String,
    #[serde(default = "identity")]
    pub mount: [f64; 4],
    #[serde(default)]
    pub noise: f64,
}

fn identity() -> [f64; 4] {
    [1.0, 0.0, 0.0, 0.0]
}

/// The Dynamixel servos drydock simulates.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
pub enum ServoModel {
    #[serde(rename = "XL330-M288")]
    Xl330M288,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServoSpec {
    /// None for a servo on no joint of the scene: it answers, and holds
    /// where it's told.
    #[serde(default)]
    pub joint: Option<String>,
    pub model: ServoModel,
    pub id: u8,
    /// What its EEPROM holds already, as the robot's servos are set up:
    /// `shutdown: 52`, by rustypot's names.
    #[serde(default)]
    pub eeprom: std::collections::BTreeMap<String, i64>,
    /// Between the servo's horn and its joint, when they don't turn together.
    #[serde(default)]
    pub transmission: Transmission,
    /// What its encoder gets wrong: perfect by default.
    #[serde(default)]
    pub encoder: EncoderSettings,
}

/// The Baud Rate register's codes (XL330 e-Manual): 9600, 57 600, 115 200
/// bit/s, then 1, 2, 3 and 4 Mbit/s.
pub const SERVO_BAUDS: [f64; 7] = [9600.0, 57600.0, 115200.0, 1e6, 2e6, 3e6, 4e6];

impl SerialSpec {
    /// The Baud Rate register's code for the bus's rate.
    pub fn baud_code(&self) -> Option<u8> {
        SERVO_BAUDS.iter().position(|&b| b == self.baud).map(|c| c as u8)
    }
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

    pub fn rated_torque(&self) -> f64 {
        self.rated_torque.unwrap_or(self.peak_torque() / 3.0)
    }

    /// The `TIMEOUT` register: `timeout_ms` in its units, 0 for none.
    pub fn timeout(&self) -> u32 {
        self.timeout_ms.map_or(0, |ms| (ms / 1e3 / reg::TIMEOUT_UNIT).round().max(1.0) as u32)
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

    /// Every Damiao motor, with its bus.
    pub fn motors(&self) -> impl Iterator<Item = (&BusSpec, &MotorSpec)> {
        self.buses.iter().flat_map(|b| b.motors.iter().map(move |m| (b, m)))
    }

    /// Every joint a motor or a servo drives, CAN buses first then serial,
    /// each in the file's order: how a scenario counts them.
    pub fn joints(&self) -> Vec<String> {
        let servos = self.serial.iter().flat_map(|b| b.motors.iter().filter_map(|m| m.joint.clone()));
        self.motors().map(|(_, m)| m.joint.clone()).chain(servos).collect()
    }

    fn check(&self) -> Result<(), String> {
        if self.buses.is_empty() && self.serial.is_empty() {
            return Err("no buses: a robot has CAN `buses`, `serial` buses, or both".into());
        }
        let mut interfaces = Vec::new();
        for bus in &self.buses {
            if interfaces.contains(&&bus.can) {
                return Err(format!("{} is listed twice", bus.can));
            }
            interfaces.push(&bus.can);
            check_bus(bus).map_err(|e| format!("{}: {e}", bus.can))?;
        }
        let mut ports = Vec::new();
        for bus in &self.serial {
            if ports.contains(&&bus.port) {
                return Err(format!("{} is listed twice", bus.port.display()));
            }
            ports.push(&bus.port);
            check_serial(bus).map_err(|e| format!("{}: {e}", bus.port.display()))?;
        }
        if !self.serial.is_empty() && self.power.is_none() {
            return Err("Dynamixel servos run at 3.7 to 6 V, or from a 2S battery: say what powers them, \
                        as `power: { volts: 7.4, battery: true }`"
                .into());
        }
        let mut joints = self.joints();
        joints.sort();
        if let Some(w) = joints.windows(2).find(|w| w[0] == w[1]) {
            return Err(format!("two motors drive {}", w[0]));
        }
        if let Some(p) = self.power {
            if !(p.volts.is_finite() && p.volts > 0.0 && p.ohms.is_finite() && p.ohms >= 0.0) {
                return Err("power: volts are above zero, ohms zero or more".into());
            }
        }
        Ok(())
    }
}

/// Servo ids are 0 to 252 and apart from each other; the bus runs at a
/// rate the servos do.
fn check_serial(bus: &SerialSpec) -> Result<(), String> {
    if bus.motors.is_empty() {
        return Err("no motors".into());
    }
    if bus.baud_code().is_none() {
        return Err(format!(
            "baud {}: Dynamixel servos run at {} bit/s",
            bus.baud,
            SERVO_BAUDS.map(|b| b.to_string()).join(", ")
        ));
    }
    let mut ids = Vec::new();
    for m in &bus.motors {
        let name = format!("servo {}", m.id);
        let name = m.joint.as_deref().unwrap_or(&name);
        if m.id > 252 {
            return Err(format!("{name}'s id {} isn't 0 to 252", m.id));
        }
        if ids.contains(&m.id) {
            return Err(format!("{name}'s id {} is another device's", m.id));
        }
        ids.push(m.id);
        if !m.transmission.ratio.is_finite() || m.transmission.ratio == 0.0 || !m.transmission.offset.is_finite() {
            return Err(format!("{name}: a transmission's ratio isn't zero"));
        }
        for (item, &value) in &m.eeprom {
            crate::dynamixel::xl330::eeprom(item, value).map_err(|e| format!("{name}: eeprom: {e}"))?;
        }
    }
    if let Some(imu) = &bus.imu {
        if ids.contains(&imu.id) || imu.id > 252 {
            return Err(format!("the imu's id {} is another device's, or isn't 0 to 252", imu.id));
        }
        let n = imu.mount.iter().map(|x| x * x).sum::<f64>();
        if !(n.is_finite() && n > 0.0 && imu.noise.is_finite() && imu.noise >= 0.0) {
            return Err("the imu's mount is a quaternion (w, x, y, z), its noise zero or more".into());
        }
    }
    Ok(())
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
    if bit_rate::code(bus.fd, bus.bitrate, bus.data_bitrate).is_none() {
        let (what, rate) = if bus.fd { ("data_bitrate", bus.data_bitrate) } else { ("bitrate", bus.bitrate) };
        return Err(format!(
            "{what} {rate}: Damiao's motors run {} at {} bit/s",
            if bus.fd { "CAN FD's data phase" } else { "classic CAN" },
            bit_rate::choices(bus.fd)
        ));
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
        if [m.peak_torque, m.rated_torque, m.timeout_ms].into_iter().flatten().any(|x| x.is_nan() || x <= 0.0) {
            return Err(format!("{}: peak_torque, rated_torque and timeout_ms are above zero", m.joint));
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

#[cfg(test)]
mod tests {
    use super::*;

    fn check(yaml: &str) -> Result<(), String> {
        serde_yaml::from_str::<Robot>(yaml).unwrap().check()
    }

    #[test]
    fn buses_run_at_rates_damiaos_motors_run_at() {
        let bus = |rates: &str| {
            format!(
                "scene: a.xml\nbuses: [{{ can: vcan0, {rates} motors: [{{ joint: j, model: DM4310, id: 1, master_id: 0x11 }}] }}]"
            )
        };
        assert!(check(&bus("")).is_ok());
        assert!(check(&bus("fd: true,")).is_ok());
        assert!(check(&bus("bitrate: 800000,")).unwrap_err().contains("125000, 200000, 250000, 500000, 1000000"));
        assert!(check(&bus("fd: true, data_bitrate: 8000000,")).unwrap_err().contains("data_bitrate 8000000"));
    }
}
