//! The physics: a MuJoCo model and its data, through the C shim
//! (`shim/world.c`). It knows joints and forces, nothing of motors.

use std::ffi::{c_char, c_double, c_int, CStr, CString};
use std::path::Path;

#[repr(C)]
struct Raw {
    _private: [u8; 0],
}

extern "C" {
    fn dd_load(path: *const c_char, err: *mut c_char, len: c_int) -> *mut Raw;
    fn dd_free(w: *mut Raw);
    fn dd_joint(
        w: *const Raw,
        name: *const c_char,
        id: *mut c_int,
        qpos: *mut c_int,
        dof: *mut c_int,
        err: *mut c_char,
        len: c_int,
    ) -> c_int;
    fn dd_release_actuators(w: *mut Raw, joint: c_int) -> c_int;
    fn dd_keyframe(w: *mut Raw, name: *const c_char, err: *mut c_char, len: c_int) -> c_int;
    fn dd_time(w: *const Raw) -> c_double;
    fn dd_timestep(w: *const Raw) -> c_double;
    fn dd_q(w: *const Raw, qpos: c_int) -> c_double;
    fn dd_v(w: *const Raw, dof: c_int) -> c_double;
    fn dd_set_force(w: *mut Raw, dof: c_int, f: c_double);
    fn dd_load_torque(w: *const Raw, dof: c_int) -> c_double;
    fn dd_set_friction(w: *mut Raw, dof: c_int, frictionloss: c_double, damping: c_double);
    fn dd_step(w: *mut Raw);
}

pub struct World {
    raw: *mut Raw,
}

/// A hinge or slide joint: where its position and velocity are.
#[derive(Debug, Clone, Copy)]
pub struct Joint {
    id: c_int,
    qpos: c_int,
    dof: c_int,
}

/// Room for MuJoCo's messages.
const ERR_LEN: usize = 1000;

fn message(buf: &[u8]) -> String {
    CStr::from_bytes_until_nul(buf).map(|s| s.to_string_lossy().into_owned()).unwrap_or_default()
}

fn c_string(s: &str) -> Result<CString, String> {
    CString::new(s).map_err(|_| format!("`{s}` has a NUL in it"))
}

impl World {
    pub fn load(path: &Path) -> Result<Self, String> {
        let c_path = c_string(&path.to_string_lossy())?;
        let mut err = [0u8; ERR_LEN];
        // SAFETY: both pointers live through the call; the shim writes at
        // most `len` bytes, NUL included.
        let raw = unsafe { dd_load(c_path.as_ptr(), err.as_mut_ptr().cast(), ERR_LEN as c_int) };
        if raw.is_null() {
            return Err(format!("can't load {}: {}", path.display(), message(&err)));
        }
        Ok(Self { raw })
    }

    pub fn joint(&self, name: &str) -> Result<Joint, String> {
        let c_name = c_string(name)?;
        let mut err = [0u8; ERR_LEN];
        let (mut id, mut qpos, mut dof) = (0, 0, 0);
        // SAFETY: as in `load`; `raw` is valid while `self` lives.
        let failed = unsafe {
            dd_joint(self.raw, c_name.as_ptr(), &mut id, &mut qpos, &mut dof, err.as_mut_ptr().cast(), ERR_LEN as c_int)
        };
        if failed != 0 {
            return Err(message(&err));
        }
        Ok(Joint { id, qpos, dof })
    }

    /// Silences the scene's own actuators on `joint`, and says how many.
    pub fn release_actuators(&mut self, joint: Joint) -> usize {
        // SAFETY: `raw` is valid; `joint` came from this world.
        unsafe { dd_release_actuators(self.raw, joint.id) as usize }
    }

    pub fn keyframe(&mut self, name: &str) -> Result<(), String> {
        let c_name = c_string(name)?;
        let mut err = [0u8; ERR_LEN];
        // SAFETY: as in `load`.
        let failed = unsafe { dd_keyframe(self.raw, c_name.as_ptr(), err.as_mut_ptr().cast(), ERR_LEN as c_int) };
        if failed != 0 {
            return Err(message(&err));
        }
        Ok(())
    }

    /// Simulated time, in seconds.
    pub fn time(&self) -> f64 {
        // SAFETY: `raw` is valid while `self` lives.
        unsafe { dd_time(self.raw) }
    }

    /// One physics step, in seconds: the scene's `<option timestep>`.
    pub fn timestep(&self) -> f64 {
        // SAFETY: as above.
        unsafe { dd_timestep(self.raw) }
    }

    /// A joint's position and velocity.
    pub fn state(&self, joint: Joint) -> (f64, f64) {
        // SAFETY: as above; the indices came from MuJoCo for this model.
        unsafe { (dd_q(self.raw, joint.qpos), dd_v(self.raw, joint.dof)) }
    }

    /// The force (or torque) on a joint until it is set again.
    pub fn set_force(&mut self, joint: Joint, f: f64) {
        // SAFETY: as above.
        unsafe { dd_set_force(self.raw, joint.dof, f) }
    }

    /// What loads a joint, besides its motor and its own friction: gravity,
    /// the other bodies' motion, contacts and limits, as of the last step.
    pub fn load_torque(&self, joint: Joint) -> f64 {
        // SAFETY: as above.
        unsafe { dd_load_torque(self.raw, joint.dof) }
    }

    /// A joint's dry friction and viscous damping, from the next step on.
    pub fn set_friction(&mut self, joint: Joint, frictionloss: f64, damping: f64) {
        // SAFETY: as above.
        unsafe { dd_set_friction(self.raw, joint.dof, frictionloss, damping) }
    }

    pub fn step(&mut self) {
        // SAFETY: as above.
        unsafe { dd_step(self.raw) }
    }
}

impl Drop for World {
    fn drop(&mut self) {
        // SAFETY: `raw` came from `dd_load` and is freed once.
        unsafe { dd_free(self.raw) }
    }
}
