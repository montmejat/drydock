//! The Microduck's body IMU: an LSM6DSV16X on a small board ("imu_to_dxl")
//! that answers on the servos' bus as one more Dynamixel device, id 200.
//!
//! Pollen hasn't published the board's firmware or its register map. What
//! drydock gives is what Pollen's runtime reads of it (`duck-control`'s
//! `imu.rs` and `bus.rs`), and what a community copy of the board documents
//! for the rest (jyg9/microduck-community-kit, `imu_to_dxl_protocol.md`):
//!
//! - From address 124, twelve bytes, read with the servos each tick: the
//!   gyro, x, y, z as `i16` at 17.5 mdps a count; then the chip's fused
//!   orientation (SFLP), x, y, z of a unit quaternion as half-floats, its w
//!   left for the reader to derive (`w = √(1 − x² − y² − z²)`, so w ≥ 0).
//! - Then eight more (the community copy's): the accelerometer, x, y, z as
//!   `i16` at 0.122 mg a count, a counter at 100 Hz, and flags (bit 0, the
//!   quaternion is valid; bit 7, fusion is running).
//! - Its model number and firmware version are the community copy's (1200,
//!   0x10); the runtime doesn't read them.
//!
//! The board's axes are the trunk's turned by its mounting, which the
//! runtime undoes (`trunk = mount · board`): the robot file gives the mount.
//! Writes are refused: the runtime never writes to it, and what the board
//! would do is unknown.

use crate::dynamixel::xl330::Refusal;
use crate::effects::Rng;
use crate::world::Inertial;

pub const MODEL_NUMBER: u16 = 1200;
pub const FIRMWARE_VERSION: u8 = 0x10;
/// The block the runtime reads, and the end of what drydock answers.
pub const BLOCK: u16 = 124;
const END: u16 = 148;
/// Gyro and accelerometer scales: rad/s and m/s² per count.
const GYRO_PER_COUNT: f64 = 0.0175 * std::f64::consts::PI / 180.0;
const ACCEL_PER_COUNT: f64 = 0.122e-3 * 9.80665;

#[derive(Debug, Clone)]
pub struct ImuBoard {
    id: u8,
    baud: u8,
    /// The board's frame, in the trunk's: w, x, y, z.
    mount: [f64; 4],
    /// The gyro's noise, rad/s (standard deviation), drawn each reading.
    noise: f64,
    rng: Rng,
    table: [u8; END as usize],
}

impl ImuBoard {
    pub fn new(id: u8, baud: u8, mount: [f64; 4], noise: f64, seed: u64) -> Self {
        let mut table = [0u8; END as usize];
        table[0..2].copy_from_slice(&MODEL_NUMBER.to_le_bytes());
        table[6] = FIRMWARE_VERSION;
        table[7] = id;
        table[8] = baud;
        Self { id, baud, mount: normalize(mount), noise, rng: Rng::new(seed), table }
    }

    pub fn id(&self) -> u8 {
        self.id
    }

    pub fn baud(&self) -> u8 {
        self.baud
    }

    /// A new reading, of the trunk at `now`, where `trunk` is what an IMU
    /// aligned with it would sense.
    pub fn sense(&mut self, now: f64, trunk: Inertial) {
        let back = conjugate(self.mount);
        let mut gyro = rotate(back, trunk.gyro);
        if self.noise > 0.0 {
            for g in &mut gyro {
                *g += self.noise * self.rng.normal();
            }
        }
        let accel = rotate(back, trunk.accel);
        let mut q = multiply(trunk.quat, self.mount);
        if q[0] < 0.0 {
            q = q.map(|x| -x);
        }
        let mut block = Vec::with_capacity(20);
        for g in gyro {
            block.extend(count(g / GYRO_PER_COUNT).to_le_bytes());
        }
        for x in &q[1..] {
            block.extend(half(*x).to_le_bytes());
        }
        for a in accel {
            block.extend(count(a / ACCEL_PER_COUNT).to_le_bytes());
        }
        block.push(((now * 100.0) as u64 % 256) as u8);
        block.push(0b1000_0001);
        self.table[BLOCK as usize..BLOCK as usize + 20].copy_from_slice(&block);
    }

    /// `len` bytes from `start`; reads past what drydock answers are refused.
    pub fn read(&self, start: u16, len: u16) -> Result<Vec<u8>, Refusal> {
        if start as u32 + len as u32 > END as u32 {
            return Err(Refusal::Access);
        }
        Ok(self.table[start as usize..(start + len) as usize].to_vec())
    }
}

fn count(x: f64) -> i16 {
    x.round().clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

/// An IEEE 754 half-float, rounded to nearest, for |x| ≤ 1 (a unit
/// quaternion's parts).
fn half(x: f64) -> u16 {
    let sign = if x.is_sign_negative() { 0x8000 } else { 0 };
    let a = x.abs();
    if a < 2f64.powi(-24) {
        return sign;
    }
    let e = a.log2().floor() as i32;
    if e < -14 {
        // Subnormal: units of 2⁻²⁴.
        return sign | (a / 2f64.powi(-24)).round() as u16;
    }
    let m = (a / 2f64.powi(e) - 1.0) * 1024.0;
    let mut bits = (((e + 15) as u16) << 10) + m.round() as u16;
    if m.round() as u16 == 1024 {
        bits = ((e + 16) as u16) << 10;
    }
    sign | bits
}

fn normalize(q: [f64; 4]) -> [f64; 4] {
    let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
    q.map(|x| x / n)
}

fn conjugate(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// The Hamilton product `a ⊗ b`.
fn multiply(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// `v` turned by the unit quaternion `q`.
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let p = multiply(multiply(q, [0.0, v[0], v[1], v[2]]), conjugate(q));
    [p[1], p[2], p[3]]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The zero3 board's mount: +90° about y (Pollen's `imu.rs`).
    const ZERO3: [f64; 4] = [std::f64::consts::FRAC_1_SQRT_2, 0.0, std::f64::consts::FRAC_1_SQRT_2, 0.0];

    fn i16_at(b: &[u8], i: usize) -> i16 {
        i16::from_le_bytes([b[i], b[i + 1]])
    }

    fn from_half(h: u16) -> f64 {
        let sign = if h & 0x8000 != 0 { -1.0 } else { 1.0 };
        let (e, m) = ((h >> 10) & 0x1F, (h & 0x3FF) as f64);
        sign * if e == 0 { m * 2f64.powi(-24) } else { (1.0 + m / 1024.0) * 2f64.powi(e as i32 - 15) }
    }

    #[test]
    fn half_floats_round_trip_to_their_precision() {
        for x in [0.0, 1.0, -1.0, 0.5, 0.7, -0.3333, 1e-5, 0.999] {
            assert!((from_half(half(x)) - x).abs() <= x.abs() * 1e-3 + 1e-7, "{x}");
        }
        assert_eq!(half(1.0), 0x3C00);
        assert_eq!(half(-2.0f64.powi(-14)), 0x8400);
    }

    /// What Pollen's runtime does with a block: the trunk's gyro, and its
    /// orientation (`imu.rs`: `gyro_trunk = rotate(mount, gyro_raw)`,
    /// `q_trunk = q_sflp ⊗ mount⁻¹`).
    fn decode(block: &[u8], mount: [f64; 4]) -> ([f64; 3], [f64; 4]) {
        let raw = [0, 2, 4].map(|i| i16_at(block, i) as f64 * GYRO_PER_COUNT);
        let [x, y, z] = [6, 8, 10].map(|i| from_half(u16::from_le_bytes([block[i], block[i + 1]])));
        let w = (1.0 - x * x - y * y - z * z).max(0.0).sqrt();
        (rotate(mount, raw), multiply([w, x, y, z], conjugate(mount)))
    }

    #[test]
    fn the_runtime_reads_back_the_trunks_motion_through_the_mount() {
        let mut board = ImuBoard::new(200, 3, ZERO3, 0.0, 1);
        // Pitched 0.2 rad nose down, turning about its own x and z.
        let quat = [(0.1f64).cos(), 0.0, (0.1f64).sin(), 0.0];
        board.sense(0.5, Inertial { quat, gyro: [0.5, 0.0, -1.0], accel: [0.0, 0.0, 9.81] });
        let block = board.read(BLOCK, 12).unwrap();
        let (gyro, q) = decode(&block, ZERO3);
        for (got, want) in gyro.iter().zip([0.5, 0.0, -1.0]) {
            assert!((got - want).abs() < 2.0 * GYRO_PER_COUNT, "{gyro:?}");
        }
        let same = q.iter().zip(quat).all(|(a, b)| (a - b).abs() < 2e-3)
            || q.iter().zip(quat).all(|(a, b)| (a + b).abs() < 2e-3);
        assert!(same, "{q:?} {quat:?}");
        // The raw gyro as Pollen's mount implies: [-ωz, ωy, ωx].
        assert_eq!(i16_at(&block, 0), count(1.0 / GYRO_PER_COUNT));
        assert_eq!(i16_at(&block, 4), count(0.5 / GYRO_PER_COUNT));
    }

    #[test]
    fn the_rest_of_the_block_is_the_accelerometer_a_counter_and_flags() {
        let mut board = ImuBoard::new(200, 3, [1.0, 0.0, 0.0, 0.0], 0.0, 1);
        board.sense(0.25, Inertial { quat: [1.0, 0.0, 0.0, 0.0], gyro: [0.0; 3], accel: [0.0, 0.0, 9.80665] });
        let b = board.read(BLOCK, 20).unwrap();
        assert_eq!(i16_at(&b, 16), count(1.0 / 0.122e-3), "1 g on z");
        assert_eq!((b[18], b[19]), (25, 0x81));
        assert_eq!(board.read(0, 7).unwrap(), [0xB0, 0x04, 0, 0, 0, 0, 0x10]);
        assert_eq!(board.read(140, 10), Err(Refusal::Access));
    }

    #[test]
    fn noise_changes_the_gyro_reading_by_reading() {
        let mut board = ImuBoard::new(200, 3, ZERO3, 0.002, 7);
        let still = Inertial { quat: [1.0, 0.0, 0.0, 0.0], gyro: [0.0; 3], accel: [0.0, 0.0, 9.81] };
        board.sense(0.0, still);
        let a = board.read(BLOCK, 6).unwrap();
        board.sense(0.02, still);
        assert_ne!(a, board.read(BLOCK, 6).unwrap());
    }
}
