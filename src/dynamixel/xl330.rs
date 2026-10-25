//! One Dynamixel XL330's firmware: its control table, what reads and writes
//! of it do, and the torque its position loop gives.
//!
//! The control table, its units, defaults and ranges are ROBOTIS's (the
//! XL330-M288 e-Manual). The position loop and the motor are BAM's model of
//! the XL330 (Rhoban's "Better Actuator Models", `bam/dynamixel/actuator.py`
//! and `bam/params/xl330/m6.json`), measured on real servos, and the model
//! Pollen trains the Microduck's policies against: a duty cycle from the
//! position error and the firmware's P gain, within the PWM and current
//! limits, then a DC motor with back-EMF, and friction that grows with the
//! load on the gearbox.
//!
//! Position control (operating mode 3) is simulated; the other modes, the
//! I and D gains, feedforward, profiles, the bus watchdog and indirect
//! addresses are kept as written but change nothing (`tests/dynamixel.rs`
//! lists them).

use std::f64::consts::TAU;

/// The control table's addresses, as ROBOTIS numbers them.
pub mod addr {
    pub const MODEL_NUMBER: u16 = 0;
    pub const FIRMWARE_VERSION: u16 = 6;
    pub const ID: u16 = 7;
    pub const BAUD_RATE: u16 = 8;
    pub const RETURN_DELAY_TIME: u16 = 9;
    pub const OPERATING_MODE: u16 = 11;
    pub const HOMING_OFFSET: u16 = 20;
    pub const MOVING_THRESHOLD: u16 = 24;
    pub const MAX_VOLTAGE_LIMIT: u16 = 32;
    pub const MIN_VOLTAGE_LIMIT: u16 = 34;
    pub const PWM_LIMIT: u16 = 36;
    pub const CURRENT_LIMIT: u16 = 38;
    pub const MAX_POSITION_LIMIT: u16 = 48;
    pub const MIN_POSITION_LIMIT: u16 = 52;
    pub const SHUTDOWN: u16 = 63;
    pub const TORQUE_ENABLE: u16 = 64;
    pub const STATUS_RETURN_LEVEL: u16 = 68;
    pub const HARDWARE_ERROR_STATUS: u16 = 70;
    pub const POSITION_P_GAIN: u16 = 84;
    pub const GOAL_PWM: u16 = 100;
    pub const GOAL_CURRENT: u16 = 102;
    pub const GOAL_POSITION: u16 = 116;
    pub const REALTIME_TICK: u16 = 120;
    pub const MOVING: u16 = 122;
    pub const PRESENT_PWM: u16 = 124;
    pub const PRESENT_CURRENT: u16 = 126;
    pub const PRESENT_VELOCITY: u16 = 128;
    pub const PRESENT_POSITION: u16 = 132;
    pub const PRESENT_INPUT_VOLTAGE: u16 = 144;
    pub const PRESENT_TEMPERATURE: u16 = 146;
    /// The end of the table: indirect data 28 is the last byte.
    pub const END: u16 = 252;
}

/// The bits of Hardware Error Status, and of Shutdown, which says which of
/// them stop the servo.
pub mod hardware_error {
    pub const INPUT_VOLTAGE: u8 = 1 << 0;
    pub const OVERHEATING: u8 = 1 << 2;
    pub const ELECTRICAL_SHOCK: u8 = 1 << 4;
    pub const OVERLOAD: u8 = 1 << 5;
}

/// Why a write was refused, as the status packet's error numbers say it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    /// A read-only item, an EEPROM item while torque is on, or outside the
    /// table (error 7).
    Access,
    /// A value out of the item's range (error 4).
    DataRange,
    /// A write that covers part of an item (error 5).
    DataLength,
    /// A goal beyond the limits set for it (error 6).
    DataLimit,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Area {
    Eeprom,
    Ram,
}

/// One item of the control table: where, how wide, whether it's written,
/// and the values it takes (inclusive).
struct Item {
    addr: u16,
    size: u16,
    writable: bool,
    area: Area,
    default: i64,
    range: (i64, i64),
}

const fn item(addr: u16, size: u16, writable: bool, area: Area, default: i64, range: (i64, i64)) -> Item {
    Item { addr, size, writable, area, default, range }
}

use Area::{Eeprom, Ram};

/// The XL330-M288's control table (ROBOTIS e-Manual). Read-only items that
/// the firmware computes (present values, the tick) are filled in on each
/// read. Goal PWM and Goal Current default to their limits, Goal Position
/// to where the servo is when torque comes on.
const TABLE: &[Item] = &[
    item(0, 2, false, Eeprom, 1200, (1200, 1200)),
    item(2, 4, false, Eeprom, 0, (0, 0)),
    // ROBOTIS ships many versions; drydock's servos are v46, the oldest
    // that answers Fast Sync Read (which Pollen's runtime relies on).
    item(6, 1, false, Eeprom, 46, (0, 255)),
    item(7, 1, true, Eeprom, 1, (0, 252)),
    item(8, 1, true, Eeprom, 1, (0, 6)),
    item(9, 1, true, Eeprom, 250, (0, 254)),
    item(10, 1, true, Eeprom, 0, (0, 13)),
    item(11, 1, true, Eeprom, 3, (0, 16)),
    item(12, 1, true, Eeprom, 255, (0, 255)),
    item(13, 1, true, Eeprom, 2, (2, 22)),
    item(20, 4, true, Eeprom, 0, (-1_044_479, 1_044_479)),
    item(24, 4, true, Eeprom, 10, (0, 1023)),
    item(31, 1, true, Eeprom, 70, (0, 100)),
    item(32, 2, true, Eeprom, 70, (31, 70)),
    item(34, 2, true, Eeprom, 35, (31, 70)),
    item(36, 2, true, Eeprom, 885, (0, 885)),
    item(38, 2, true, Eeprom, 1750, (0, 1750)),
    item(44, 4, true, Eeprom, 445, (0, 2047)),
    item(48, 4, true, Eeprom, 4095, (0, 4095)),
    item(52, 4, true, Eeprom, 0, (0, 4095)),
    item(60, 1, true, Eeprom, 0, (0, 3)),
    item(62, 1, true, Eeprom, 140, (1, 255)),
    item(63, 1, true, Eeprom, 53, (0, 255)),
    item(64, 1, true, Ram, 0, (0, 1)),
    item(65, 1, true, Ram, 0, (0, 1)),
    item(68, 1, true, Ram, 2, (0, 2)),
    item(69, 1, false, Ram, 0, (0, 1)),
    item(70, 1, false, Ram, 0, (0, 255)),
    item(76, 2, true, Ram, 1600, (0, 16383)),
    item(78, 2, true, Ram, 180, (0, 16383)),
    item(80, 2, true, Ram, 0, (0, 16383)),
    item(82, 2, true, Ram, 0, (0, 16383)),
    item(84, 2, true, Ram, 400, (0, 16383)),
    item(88, 2, true, Ram, 0, (0, 16383)),
    item(90, 2, true, Ram, 0, (0, 16383)),
    item(98, 1, true, Ram, 0, (0, 127)),
    item(100, 2, true, Ram, 885, (-885, 885)),
    item(102, 2, true, Ram, 1750, (-1750, 1750)),
    item(104, 4, true, Ram, 0, (-2047, 2047)),
    item(108, 4, true, Ram, 0, (0, 32767)),
    item(112, 4, true, Ram, 0, (0, 32767)),
    item(116, 4, true, Ram, 2048, (0, 4095)),
    item(120, 2, false, Ram, 0, (0, 32767)),
    item(122, 1, false, Ram, 0, (0, 1)),
    item(123, 1, false, Ram, 0, (0, 255)),
    item(124, 2, false, Ram, 0, (-885, 885)),
    item(126, 2, false, Ram, 0, (-32768, 32767)),
    item(128, 4, false, Ram, 0, (i32::MIN as i64, i32::MAX as i64)),
    item(132, 4, false, Ram, 0, (i32::MIN as i64, i32::MAX as i64)),
    item(136, 4, false, Ram, 0, (i32::MIN as i64, i32::MAX as i64)),
    item(140, 4, false, Ram, 0, (i32::MIN as i64, i32::MAX as i64)),
    item(144, 2, false, Ram, 0, (0, 255)),
    item(146, 1, false, Ram, 25, (0, 255)),
    item(147, 1, false, Ram, 0, (0, 1)),
];

/// The EEPROM items a robot file may set, by the names rustypot gives them
/// (`servo/dynamixel/xl330.rs`), and where they are. Id and baud rate come
/// from the robot file's own fields.
pub const EEPROM: &[(&str, u16)] = &[
    ("return_delay_time", 9),
    ("drive_mode", 10),
    ("operating_mode", 11),
    ("secondary_id", 12),
    ("protocol_type", 13),
    ("homing_offset", 20),
    ("moving_threshold", 24),
    ("temperature_limit", 31),
    ("max_voltage_limit", 32),
    ("min_voltage_limit", 34),
    ("pwm_limit", 36),
    ("current_limit", 38),
    ("velocity_limit", 44),
    ("max_position_limit", 48),
    ("min_position_limit", 52),
    ("startup_configuration", 60),
    ("pwm_slope", 62),
    ("shutdown", 63),
];

/// An EEPROM item's address and width, by name, if `value` is within its
/// range.
pub fn eeprom(name: &str, value: i64) -> Result<(u16, u16), String> {
    let &(_, a) = EEPROM.iter().find(|(n, _)| *n == name).ok_or_else(|| {
        format!("no EEPROM item `{name}`; there are {}", EEPROM.iter().map(|(n, _)| *n).collect::<Vec<_>>().join(", "))
    })?;
    let i = find(a).expect("in the table");
    if !(i.range.0..=i.range.1).contains(&value) {
        return Err(format!("{name} takes {} to {}, not {value}", i.range.0, i.range.1));
    }
    Ok((i.addr, i.size))
}

fn find(addr: u16) -> Option<&'static Item> {
    TABLE.iter().find(|i| (i.addr..i.addr + i.size).contains(&addr))
}

/// Indirect addresses 1 to 28 (2 bytes each) and their data (1 byte each):
/// kept as written; reading the data doesn't follow the addresses yet.
fn indirect(addr: u16) -> bool {
    (168..addr::END).contains(&addr)
}

/// BAM's XL330 (`m6.json` and `XL330Actuator`).
pub mod bam {
    /// Torque constant, N m/A, or back-EMF, V/(rad/s).
    pub const KT: f64 = 0.36601349688984386;
    /// Winding resistance, Ω.
    pub const R: f64 = 2.8113923539223227;
    /// Pulses per turn, and the firmware's P divisor ("empirically observed
    /// for XL330 (manual mentions 128)").
    pub const PULSES: f64 = 4096.0;
    pub const KP_DIVISOR: f64 = 256.0;
    /// The PWM limit's full scale: 885 is 100%.
    pub const PWM_FULL: f64 = 885.0;
    /// Friction (m6: Stribeck, load-dependent, directional, quadratic).
    pub const FRICTION_BASE: f64 = 0.004771183165566;
    pub const FRICTION_STRIBECK: f64 = 0.004676345799486616;
    pub const LOAD_FRICTION_MOTOR: f64 = 0.2667860954283698;
    pub const LOAD_FRICTION_EXTERNAL: f64 = 8.515871897059342e-06;
    pub const LOAD_FRICTION_MOTOR_STRIBECK: f64 = 1.0722918395099123e-05;
    pub const LOAD_FRICTION_EXTERNAL_STRIBECK: f64 = 0.08077928978935671;
    pub const LOAD_FRICTION_MOTOR_QUAD: f64 = 0.009972471242139415;
    pub const LOAD_FRICTION_EXTERNAL_QUAD: f64 = 0.004902565732332559;
    pub const DTHETA_STRIBECK: f64 = 2.890372094130307;
    pub const ALPHA: f64 = 8.683259907618984;
    pub const FRICTION_VISCOUS: f64 = 0.005359668274599504;

    /// The friction budget: the dry friction (N m) a joint can resist with,
    /// given the motor's torque and the load's (gravity, contacts...), and
    /// its speed. MuJoCo's solver clips the friction to it, so a held joint
    /// sticks. BAM's `compute_frictions`, for m6.
    pub fn frictionloss(motor: f64, external: f64, speed: f64) -> f64 {
        let gearbox = (external * LOAD_FRICTION_EXTERNAL - motor * LOAD_FRICTION_MOTOR).abs();
        let gearbox_stribeck =
            (external * LOAD_FRICTION_EXTERNAL_STRIBECK - motor * LOAD_FRICTION_MOTOR_STRIBECK).abs();
        let stribeck = (-(speed / DTHETA_STRIBECK).abs().powf(ALPHA)).exp();
        let mut f = FRICTION_BASE + gearbox + stribeck * (FRICTION_STRIBECK + gearbox_stribeck);
        if external.signum() != motor.signum() {
            // The quadratic terms: one or the other, whichever drives.
            let quad = if external.abs() < motor.abs() {
                LOAD_FRICTION_EXTERNAL_QUAD * external * external
            } else if external.abs() > motor.abs() {
                LOAD_FRICTION_MOTOR_QUAD * motor * motor
            } else {
                0.0
            };
            f += stribeck * quad;
        }
        f
    }
}

/// Radians per pulse, and pulses at the horn's centre: the Microduck's
/// runtime takes position 2048 as 0 rad (`rad = 2π·raw/4096 − π`).
const RAD_PER_PULSE: f64 = TAU / bam::PULSES;
const CENTRE: f64 = 2048.0;
/// Velocity units, rad/s: 0.229 rev/min.
const RAD_S_PER_UNIT: f64 = 0.229 * TAU / 60.0;

/// What the servo's loop gives the joint this step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Output {
    /// N m, at the horn.
    pub torque: f64,
    /// The dry friction the gearbox resists with, N m, and its viscous
    /// friction, N m s/rad: the joint's, until the next step.
    pub frictionloss: f64,
    pub damping: f64,
    /// The motor's current, A, and the duty cycle (−1 to 1).
    pub current: f64,
    pub duty: f64,
}

#[derive(Debug, Clone)]
pub struct Xl330 {
    /// The joint it drives, in the scene.
    pub joint: String,
    table: [u8; addr::END as usize],
    /// What the firmware last computed, for the present values.
    output: Output,
    /// Its horn's angle (rad, 0 at the centre) and speed, as last seen.
    angle: f64,
    speed: f64,
    /// The supply's voltage at the servo.
    volts: f64,
    /// The time, for the realtime tick.
    now: f64,
}

impl Xl330 {
    /// As it leaves the factory, but at `id` and talking at `baud` (a Baud
    /// Rate code), as the robot file says it's set.
    pub fn new(joint: String, id: u8, baud: u8) -> Self {
        let mut table = [0u8; addr::END as usize];
        for i in TABLE {
            put(&mut table, i.addr, i.size, i.default);
        }
        put(&mut table, addr::ID, 1, id as i64);
        put(&mut table, addr::BAUD_RATE, 1, baud as i64);
        let output = Output { torque: 0.0, frictionloss: bam::FRICTION_BASE, damping: 0.0, current: 0.0, duty: 0.0 };
        Self { joint, table, output, angle: 0.0, speed: 0.0, volts: 0.0, now: 0.0 }
    }

    fn get(&self, addr: u16) -> i64 {
        let i = find(addr).expect("an item of the table");
        get(&self.table, i.addr, i.size)
    }

    pub fn id(&self) -> u8 {
        self.get(addr::ID) as u8
    }

    /// The Baud Rate code it talks at.
    pub fn baud(&self) -> u8 {
        self.get(addr::BAUD_RATE) as u8
    }

    /// What its EEPROM holds already, as a robot's servos are set up: by
    /// name, values [`eeprom`] took.
    pub fn preset(&mut self, name: &str, value: i64) -> Result<(), String> {
        let (a, size) = eeprom(name, value)?;
        put(&mut self.table, a, size, value);
        Ok(())
    }

    /// A step for a servo on no joint (the Microduck's mouth, which its
    /// model leaves out): with torque on, its horn is where it's told.
    pub fn idle(&mut self, now: f64, volts: f64) {
        let goal = self.get(addr::GOAL_POSITION) as f64 - self.get(addr::HOMING_OFFSET) as f64;
        let angle = if self.torque_enabled() { (goal - CENTRE) * RAD_PER_PULSE } else { self.angle };
        self.step(now, angle, 0.0, volts, 0.0);
    }

    /// How long it waits before answering, seconds.
    pub fn return_delay(&self) -> f64 {
        self.get(addr::RETURN_DELAY_TIME) as f64 * 2e-6
    }

    /// 0: it answers pings only; 1: pings and reads; 2: everything.
    pub fn status_return_level(&self) -> u8 {
        self.get(addr::STATUS_RETURN_LEVEL) as u8
    }

    pub fn torque_enabled(&self) -> bool {
        self.get(addr::TORQUE_ENABLE) == 1
    }

    pub fn hardware_error(&self) -> u8 {
        self.get(addr::HARDWARE_ERROR_STATUS) as u8
    }

    pub fn operating_mode(&self) -> u8 {
        self.get(addr::OPERATING_MODE) as u8
    }

    /// `len` bytes from `start`, the present values as of the last step.
    /// Gaps between items read 0. Reads past the table are refused.
    pub fn read(&mut self, start: u16, len: u16) -> Result<Vec<u8>, Refusal> {
        if start as u32 + len as u32 > addr::END as u32 {
            return Err(Refusal::Access);
        }
        self.present();
        Ok(self.table[start as usize..(start + len) as usize].to_vec())
    }

    /// `data` from `start`. Every item it touches must be written whole,
    /// writable (EEPROM only with torque off), and within its range; then
    /// all of it is written, or none. Turning torque on takes the present
    /// position as the goal, as the real servo's position loop starts where
    /// it is.
    pub fn write(&mut self, start: u16, data: &[u8]) -> Result<(), Refusal> {
        self.check(start, data)?;
        let end = start as usize + data.len();
        let torque_on = !self.torque_enabled()
            && (start as usize..end).contains(&(addr::TORQUE_ENABLE as usize))
            && data[(addr::TORQUE_ENABLE - start) as usize] == 1;
        self.table[start as usize..end].copy_from_slice(data);
        if torque_on {
            let here = self.actual_position() + self.get(addr::HOMING_OFFSET) as f64;
            put(&mut self.table, addr::GOAL_POSITION, 4, here.round() as i64);
        }
        Ok(())
    }

    /// Whether `write` would take `data` at `start`, writing nothing.
    pub fn check(&self, start: u16, data: &[u8]) -> Result<(), Refusal> {
        let end = start as u32 + data.len() as u32;
        if end > addr::END as u32 {
            return Err(Refusal::Access);
        }
        let mut a = start;
        while (a as u32) < end {
            if indirect(a) {
                a += 1;
                continue;
            }
            let Some(i) = find(a) else {
                a += 1;
                continue;
            };
            if i.addr < start || (i.addr + i.size) as u32 > end {
                return Err(Refusal::DataLength);
            }
            if !i.writable || (i.area == Area::Eeprom && self.torque_enabled()) {
                return Err(Refusal::Access);
            }
            let off = (i.addr - start) as usize;
            let value = get(&data[off..], 0, i.size);
            if !(i.range.0..=i.range.1).contains(&value) {
                return Err(Refusal::DataRange);
            }
            if i.addr == addr::GOAL_POSITION {
                let (lo, hi) = (self.get(addr::MIN_POSITION_LIMIT), self.get(addr::MAX_POSITION_LIMIT));
                if !(lo..=hi).contains(&value) {
                    return Err(Refusal::DataLimit);
                }
            }
            a = i.addr + i.size;
        }
        Ok(())
    }

    /// A reboot: RAM back to its defaults (torque off, errors cleared);
    /// EEPROM kept.
    pub fn reboot(&mut self) {
        for i in TABLE.iter().filter(|i| i.area == Area::Ram) {
            put(&mut self.table, i.addr, i.size, i.default);
        }
        self.table[168..addr::END as usize].fill(0);
        self.output.torque = 0.0;
    }

    /// Factory Reset: everything back as the factory set it, but its id
    /// with `option` 0x01, its id and baud rate with 0x02 (the spec). A
    /// servo reset to another baud rate than the bus's stops hearing it.
    pub fn factory_reset(&mut self, option: u8) {
        let (id, baud) = (self.id(), self.baud());
        for i in TABLE {
            put(&mut self.table, i.addr, i.size, i.default);
        }
        self.table[168..addr::END as usize].fill(0);
        if option == 0x01 || option == 0x02 {
            put(&mut self.table, addr::ID, 1, id as i64);
        }
        if option == 0x02 {
            put(&mut self.table, addr::BAUD_RATE, 1, baud as i64);
        }
        self.output.torque = 0.0;
    }

    /// The position the servo reads, in pulses, before its homing offset.
    fn actual_position(&self) -> f64 {
        CENTRE + self.angle / RAD_PER_PULSE
    }

    /// One step of its position loop at `now`, the horn at `angle` (rad),
    /// turning at `speed` (rad/s), on `volts`, with `load` (N m) on it from
    /// gravity, contacts and the rest. BAM's `compute_control`, then
    /// `compute_torque`, then the friction budget, which uses the motor's
    /// torque of the step before, as BAM does.
    pub fn step(&mut self, now: f64, angle: f64, speed: f64, volts: f64, load: f64) -> Output {
        (self.now, self.angle, self.speed, self.volts) = (now, angle, speed, volts);
        self.check_voltage();
        let previous = self.output.torque;
        let (duty, torque) = if self.torque_enabled() && self.operating_mode() == 3 {
            let goal = self.get(addr::GOAL_POSITION) as f64 - self.get(addr::HOMING_OFFSET) as f64;
            let error = goal - self.actual_position();
            let kp = self.get(addr::POSITION_P_GAIN) as f64;
            let mut duty = error * kp / bam::KP_DIVISOR / bam::PWM_FULL;
            // The firmware's current limit, as a window on the duty cycle; then
            // the PWM limit, which the battery can't exceed.
            let i_max = self.get(addr::CURRENT_LIMIT) as f64 / 1e3;
            let emf = bam::KT * speed;
            if volts > 0.0 {
                duty = duty.clamp((emf - bam::R * i_max) / volts, (emf + bam::R * i_max) / volts);
            }
            let pwm = self.get(addr::PWM_LIMIT).min(self.get(addr::GOAL_PWM).abs()) as f64 / bam::PWM_FULL;
            let duty = duty.clamp(-pwm, pwm);
            (duty, bam::KT * (duty * volts - emf) / bam::R)
        } else {
            (0.0, 0.0)
        };
        let current = if torque == 0.0 { 0.0 } else { (duty * volts - bam::KT * speed) / bam::R };
        let frictionloss = bam::frictionloss(previous, load, speed);
        self.output = Output { torque, frictionloss, damping: bam::FRICTION_VISCOUS, current, duty };
        self.output
    }

    /// Outside its voltage limits, the input voltage error, if Shutdown
    /// includes it: then torque goes off, and the error stays, until a
    /// reboot. With the bit clear, as the Microduck sets it to run its
    /// servos from a 2S battery above their 7.0 V limit, nothing is flagged:
    /// Pollen's runtime sees no alert then. ROBOTIS doesn't say whether an
    /// error outside Shutdown is flagged; assumed not.
    fn check_voltage(&mut self) {
        let (lo, hi) =
            (self.get(addr::MIN_VOLTAGE_LIMIT) as f64 / 10.0, self.get(addr::MAX_VOLTAGE_LIMIT) as f64 / 10.0);
        let shutdown = self.get(addr::SHUTDOWN) as u8;
        if (self.volts < lo || self.volts > hi) && shutdown & hardware_error::INPUT_VOLTAGE != 0 {
            let errors = self.hardware_error() | hardware_error::INPUT_VOLTAGE;
            put(&mut self.table, addr::HARDWARE_ERROR_STATUS, 1, errors as i64);
            put(&mut self.table, addr::TORQUE_ENABLE, 1, 0);
        }
    }

    /// The present values, into the table, from the last step.
    fn present(&mut self) {
        let o = self.output;
        let position = self.actual_position() + self.get(addr::HOMING_OFFSET) as f64;
        let velocity = self.speed / RAD_S_PER_UNIT;
        let moving = velocity.abs() > self.get(addr::MOVING_THRESHOLD) as f64;
        let t = &mut self.table;
        put(t, addr::REALTIME_TICK, 2, ((self.now * 1e3) as i64).rem_euclid(32768));
        put(t, addr::MOVING, 1, moving as i64);
        put(t, addr::PRESENT_PWM, 2, (o.duty * bam::PWM_FULL).round() as i64);
        put(t, addr::PRESENT_CURRENT, 2, (o.current * 1e3).round() as i64);
        put(t, addr::PRESENT_VELOCITY, 4, velocity.round() as i64);
        put(t, addr::PRESENT_POSITION, 4, position.round() as i64);
        put(t, addr::PRESENT_INPUT_VOLTAGE, 2, (self.volts * 10.0).round() as i64);
    }
}

/// A little-endian value of `size` bytes at `addr`, signed if its item is.
fn get(table: &[u8], addr: u16, size: u16) -> i64 {
    let b = &table[addr as usize..(addr + size) as usize];
    match size {
        1 => b[0] as i64,
        2 => i16::from_le_bytes([b[0], b[1]]) as i64,
        _ => i32::from_le_bytes([b[0], b[1], b[2], b[3]]) as i64,
    }
}

fn put(table: &mut [u8], addr: u16, size: u16, value: i64) {
    let bytes = (value as i32).to_le_bytes();
    table[addr as usize..(addr + size) as usize].copy_from_slice(&bytes[..size as usize]);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn servo() -> Xl330 {
        Xl330::new("j".into(), 10, 3)
    }

    fn u32_at(s: &mut Xl330, a: u16) -> i32 {
        let b = s.read(a, 4).unwrap();
        i32::from_le_bytes([b[0], b[1], b[2], b[3]])
    }

    #[test]
    fn its_table_starts_as_the_factory_set_it() {
        let mut s = servo();
        assert_eq!(s.read(addr::MODEL_NUMBER, 2).unwrap(), 1200u16.to_le_bytes());
        assert_eq!((s.id(), s.baud(), s.return_delay()), (10, 3, 500e-6));
        assert_eq!(s.read(addr::POSITION_P_GAIN, 2).unwrap(), 400u16.to_le_bytes());
        assert_eq!(s.read(addr::SHUTDOWN, 1).unwrap(), [53]);
    }

    #[test]
    fn writes_are_checked_item_by_item() {
        let mut s = servo();
        assert_eq!(s.write(addr::MODEL_NUMBER, &[0, 0]), Err(Refusal::Access));
        assert_eq!(s.write(addr::POSITION_P_GAIN, &[0]), Err(Refusal::DataLength));
        assert_eq!(s.write(addr::BAUD_RATE, &[7]), Err(Refusal::DataRange));
        assert_eq!(s.write(addr::RETURN_DELAY_TIME, &[0]), Ok(()));
        s.write(addr::TORQUE_ENABLE, &[1]).unwrap();
        assert_eq!(s.write(addr::RETURN_DELAY_TIME, &[5]), Err(Refusal::Access), "EEPROM, with torque on");
        assert_eq!(s.write(addr::GOAL_POSITION, &5000i32.to_le_bytes()), Err(Refusal::DataRange));
        s.write(addr::TORQUE_ENABLE, &[0]).unwrap();
        s.write(addr::MAX_POSITION_LIMIT, &3000i32.to_le_bytes()).unwrap();
        s.write(addr::TORQUE_ENABLE, &[1]).unwrap();
        assert_eq!(s.write(addr::GOAL_POSITION, &3500i32.to_le_bytes()), Err(Refusal::DataLimit));
    }

    #[test]
    fn torque_on_holds_where_it_is_and_pulls_towards_the_goal() {
        let mut s = servo();
        s.step(0.0, 0.3, 0.0, 5.0, 0.0);
        s.write(addr::TORQUE_ENABLE, &[1]).unwrap();
        let here = u32_at(&mut s, addr::GOAL_POSITION);
        assert_eq!(here, (2048.0 + 0.3 / RAD_PER_PULSE).round() as i32);
        assert!(s.step(0.0, 0.3, 0.0, 5.0, 0.0).torque.abs() < 1e-2);
        // 0.1 rad short of the goal, at P 400: about 15% duty, 0.15 N m.
        let out = s.step(0.0, 0.2, 0.0, 5.0, 0.0);
        let duty = (here as f64 - (2048.0 + 0.2 / RAD_PER_PULSE)) * 400.0 / 256.0 / 885.0;
        assert!((out.duty - duty).abs() < 1e-9);
        assert!((out.torque - bam::KT * duty * 5.0 / bam::R).abs() < 1e-9);
    }

    #[test]
    fn its_current_limit_caps_the_torque() {
        let mut s = servo();
        s.write(addr::TORQUE_ENABLE, &[1]).unwrap();
        let out = s.step(0.0, -1.5, 0.0, 5.0, 0.0);
        assert!((out.current - 1.75).abs() < 1e-9, "{out:?}");
        assert!((out.torque - bam::KT * 1.75).abs() < 1e-9);
    }

    #[test]
    fn over_its_voltage_limit_it_shuts_down_unless_shutdown_says_not_to() {
        let mut s = servo();
        s.write(addr::TORQUE_ENABLE, &[1]).unwrap();
        s.step(0.0, 0.0, 0.0, 7.4, 0.0);
        assert_eq!(s.hardware_error(), hardware_error::INPUT_VOLTAGE);
        assert!(!s.torque_enabled(), "Shutdown 53 includes the input voltage bit");
        // The Microduck sets Shutdown 52: flagged, but it keeps going.
        s.reboot();
        s.write(addr::SHUTDOWN, &[52]).unwrap();
        s.write(addr::TORQUE_ENABLE, &[1]).unwrap();
        s.step(0.0, 0.0, 0.0, 7.4, 0.0);
        assert!(s.torque_enabled() && s.hardware_error() == 0);
    }

    #[test]
    fn present_values_read_in_the_tables_units() {
        let mut s = servo();
        s.write(addr::HOMING_OFFSET, &(-512i32).to_le_bytes()).unwrap();
        s.step(0.25, 0.0, 1.0, 7.4, 0.0);
        assert_eq!(u32_at(&mut s, addr::PRESENT_POSITION), 2048 - 512);
        assert_eq!(u32_at(&mut s, addr::PRESENT_VELOCITY), (1.0 / RAD_S_PER_UNIT).round() as i32);
        assert_eq!(s.read(addr::PRESENT_INPUT_VOLTAGE, 2).unwrap(), 74u16.to_le_bytes());
        assert_eq!(s.read(addr::REALTIME_TICK, 2).unwrap(), 250u16.to_le_bytes());
    }

    #[test]
    fn a_held_joint_sticks_harder_under_load() {
        let idle = bam::frictionloss(0.0, 0.0, 0.0);
        let loaded = bam::frictionloss(0.3, -0.3, 0.0);
        assert!((idle - bam::FRICTION_BASE - bam::FRICTION_STRIBECK).abs() < 1e-12);
        assert!(loaded > idle + 0.05, "{loaded}");
        assert!(bam::frictionloss(0.0, 0.0, 10.0) < idle, "Stribeck: less once moving");
    }
}
