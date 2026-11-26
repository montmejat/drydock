//! A CiA 402 servo drive's firmware, behind its ET1100: the EtherCAT state
//! machine (Init, Pre-Operational, Safe-Operational, Operational), its
//! object dictionary over CoE, its PDOs, and CiA 402's state machine and
//! cyclic modes (CSP, CSV, CST), driving a joint.
//!
//! Sources: ETG.1000.6 by way of SOEM's and ethercrab's state handling for
//! the AL states and their status codes; CiA 402 (IEC 61800-7-201) by way
//! of drive manuals for the state machine, the controlword and statusword
//! bits, and the objects' indices and units. Where the sources are silent,
//! drydock chooses, and says so: the loops' gains, the following error's
//! defaults, and what the drive does when it leaves Operational (it turns
//! its torque off: Switch On Disabled).

use super::coe::{self, abort, kind, Dictionary, Write};
use super::esc::{state, Esc};
use super::frame::{Access, Device, Reach};
use super::sii::{self, Areas, Identity, Pdo};

/// AL status codes (ETG.1000.6).
pub mod al_code {
    pub const INVALID_STATE_CHANGE: u16 = 0x0011;
    pub const UNKNOWN_STATE: u16 = 0x0012;
    pub const BOOTSTRAP_NOT_SUPPORTED: u16 = 0x0013;
    pub const INVALID_MAILBOX: u16 = 0x0016;
    pub const SM_WATCHDOG: u16 = 0x001B;
    pub const INVALID_OUTPUTS: u16 = 0x001D;
    pub const INVALID_INPUTS: u16 = 0x001E;
}

/// CiA 402's error codes (0x603F) the drive raises.
pub mod error_code {
    pub const FOLLOWING_ERROR: u16 = 0x8611;
    pub const SHORT_CIRCUIT: u16 = 0x2320;
}

/// The modes of operation (0x6060) the drive runs.
pub mod mode {
    pub const CSP: i8 = 8;
    pub const CSV: i8 = 9;
    pub const CST: i8 = 10;
}

/// CiA 402's states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cia402 {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
}

impl Cia402 {
    /// The statusword's state bits (0, 1, 2, 3, 5, 6).
    fn bits(self) -> u16 {
        match self {
            Cia402::NotReadyToSwitchOn => 0x0000,
            Cia402::SwitchOnDisabled => 0x0040,
            Cia402::ReadyToSwitchOn => 0x0021,
            Cia402::SwitchedOn => 0x0023,
            Cia402::OperationEnabled => 0x0027,
            Cia402::QuickStopActive => 0x0007,
            Cia402::FaultReactionActive => 0x000F,
            Cia402::Fault => 0x0008,
        }
    }
}

/// What a drive is set to, from the robot file.
#[derive(Debug, Clone)]
pub struct Settings {
    pub vendor: u32,
    pub product: u32,
    pub revision: u32,
    pub serial: u32,
    pub name: String,
    /// Encoder counts per turn of the motor (0x608F).
    pub counts_per_turn: u32,
    /// N m: 0x6076, what torques are thousandths of.
    pub rated_torque: f64,
    /// N m: what it can give at most; 0x6072 starts there.
    pub peak_torque: f64,
    /// CSP's position loop, N m/rad and N m s/rad; CSV's velocity loop, N m s/rad.
    pub kp: f64,
    pub kd: f64,
    pub kv: f64,
}

/// Where the sync managers' buffers are. The three-buffer areas are three
/// times their length in a real chip; they're far enough apart here.
const AREAS: Areas =
    Areas { mailbox_out: (0x1000, 128), mailbox_in: (0x1080, 128), outputs: (0x1100, 0), inputs: (0x1400, 0) };

/// Mapping objects' subindices: up to 8 entries each.
const MAP_ENTRIES: u8 = 8;

/// The default PDOs: what most CSP drives map, so CSV and CST need no
/// remapping. (index, subindex, bits, data type)
fn default_rx() -> Pdo {
    Pdo {
        index: 0x1600,
        entries: vec![
            (0x6040, 0, 16, kind::UNSIGNED16),
            (0x607A, 0, 32, kind::INTEGER32),
            (0x60FF, 0, 32, kind::INTEGER32),
            (0x6071, 0, 16, kind::INTEGER16),
            (0x6060, 0, 8, kind::INTEGER8),
        ],
    }
}

fn default_tx() -> Pdo {
    Pdo {
        index: 0x1A00,
        entries: vec![
            (0x6041, 0, 16, kind::UNSIGNED16),
            (0x6064, 0, 32, kind::INTEGER32),
            (0x606C, 0, 32, kind::INTEGER32),
            (0x6077, 0, 16, kind::INTEGER16),
            (0x6061, 0, 8, kind::INTEGER8),
            (0x603F, 0, 16, kind::UNSIGNED16),
        ],
    }
}

pub struct Drive {
    pub joint: String,
    pub esc: Esc,
    settings: Settings,
    od: Dictionary,
    counter: u8,
    cia: Cia402,
    last_control: u16,
    /// When the master last wrote the outputs, s (simulated time).
    outputs_at: f64,
    now: f64,
    /// Since when the following error has been outside its window.
    following_since: Option<f64>,
    /// What the drive asks of its motor, and what it got, N m.
    asked: f64,
    applied: f64,
    /// The motor's angle and speed, as its encoder last said.
    theta: f64,
    omega: f64,
    /// Whether the supply is on.
    powered: bool,
    /// The master wrote the outputs since this was last asked: a cycle.
    cycled: bool,
}

impl Drive {
    pub fn new(joint: String, settings: Settings) -> Self {
        let (rx, tx) = (default_rx(), default_tx());
        let areas = Areas {
            outputs: (AREAS.outputs.0, rx.bytes() as u16),
            inputs: (AREAS.inputs.0, tx.bytes() as u16),
            ..AREAS
        };
        let id = Identity {
            vendor: settings.vendor,
            product: settings.product,
            revision: settings.revision,
            serial: settings.serial,
            name: settings.name.clone(),
        };
        let esc = Esc::new(sii::image(&id, areas, &rx, &tx));
        let od = dictionary(&settings, &rx, &tx);
        let mut drive = Self {
            joint,
            esc,
            settings,
            od,
            counter: 0,
            cia: Cia402::SwitchOnDisabled,
            last_control: 0,
            outputs_at: 0.0,
            now: 0.0,
            following_since: None,
            asked: 0.0,
            applied: 0.0,
            theta: 0.0,
            omega: 0.0,
            powered: true,
            cycled: false,
        };
        drive.statusword();
        drive
    }

    /// Whether the master wrote its outputs since this was last asked.
    pub fn take_cycle(&mut self) -> bool {
        std::mem::take(&mut self.cycled)
    }

    pub fn state(&self) -> Cia402 {
        self.cia
    }

    pub fn al_state(&self) -> u8 {
        self.esc.al_state()
    }

    /// The position's units: counts to radians at the motor.
    fn rad(&self, counts: f64) -> f64 {
        counts * std::f64::consts::TAU / self.settings.counts_per_turn as f64
    }

    fn counts(&self, rad: f64) -> f64 {
        rad * self.settings.counts_per_turn as f64 / std::f64::consts::TAU
    }

    /// What the encoder says, at time `now`: the actual values the drive
    /// reports update from it.
    pub fn sense(&mut self, now: f64, theta: f64, omega: f64) {
        self.now = now;
        self.theta = theta;
        self.omega = omega;
        let counts = self.counts(theta).round() as i64 as i32;
        let speed = self.counts(omega).round() as i32;
        let torque = (self.applied / self.settings.rated_torque * 1000.0).round() as i16;
        self.od.set(0x6064, 0, &counts.to_le_bytes());
        self.od.set(0x606C, 0, &speed.to_le_bytes());
        self.od.set(0x6077, 0, &torque.to_le_bytes());
        let error = self.od.i32(0x607A, 0).wrapping_sub(counts);
        self.od.set(0x60F4, 0, &error.to_le_bytes());
        self.inputs();
    }

    pub fn power(&mut self, volts: f64) {
        self.powered = volts > 0.0;
    }

    /// The torque the drive's loops ask for now, N m at the motor, before
    /// its actuator limits it. Its current loop is far faster than a
    /// physics step, so the torque it asks is what it gets.
    pub fn law(&mut self) -> f64 {
        self.watchdog();
        self.following();
        let max = self.od.u16(0x6072, 0) as f64 / 1000.0 * self.settings.rated_torque;
        let s = &self.settings;
        let torque = match self.cia {
            Cia402::OperationEnabled if self.powered => match self.od.u8(0x6061, 0) as i8 {
                mode::CSP => {
                    let error = self.od.i32(0x607A, 0).wrapping_sub(self.od.i32(0x6064, 0)) as f64;
                    s.kp * self.rad(error) - s.kd * self.omega
                }
                mode::CSV => s.kv * (self.rad(self.od.i32(0x60FF, 0) as f64) - self.omega),
                mode::CST => self.od.u16(0x6071, 0) as i16 as f64 / 1000.0 * s.rated_torque,
                _ => 0.0,
            },
            // Quick stop: brake on the velocity loop, then let go once still
            // (quick stop option code 2: slow down, then Switch On Disabled).
            Cia402::QuickStopActive => {
                if self.omega.abs() < 0.01 {
                    self.to(Cia402::SwitchOnDisabled);
                    0.0
                } else {
                    -s.kv * self.omega
                }
            }
            _ => 0.0,
        };
        self.asked = torque.clamp(-max, max);
        self.asked
    }

    /// What the motor gave.
    pub fn applied(&mut self, torque: f64) {
        self.applied = torque;
    }

    /// The motor shorted: the drive trips.
    pub fn trip(&mut self, code: u16) {
        self.fault(code);
    }

    fn to(&mut self, s: Cia402) {
        self.cia = s;
        self.statusword();
    }

    fn fault(&mut self, code: u16) {
        self.od.set(0x603F, 0, &code.to_le_bytes());
        // The fault reaction is the torque turned off: it's over at once.
        self.to(Cia402::FaultReactionActive);
        self.to(Cia402::Fault);
    }

    fn statusword(&mut self) {
        let mut sw = self.cia.bits();
        if self.powered {
            sw |= 0x0010; // voltage enabled
        }
        sw |= 0x0200; // remote: the controlword is obeyed
        if self.cia == Cia402::OperationEnabled && self.od.u8(0x6061, 0) as i8 == mode::CSP {
            sw |= 0x1000; // following the target
        }
        if self.following_since.is_some_and(|t| self.now - t >= self.following_timeout()) {
            sw |= 0x2000;
        }
        self.od.set(0x6041, 0, &sw.to_le_bytes());
    }

    fn following_timeout(&self) -> f64 {
        self.od.u16(0x6066, 0) as f64 / 1e3
    }

    /// In CSP, a following error outside its window (0x6065) for longer than
    /// its timeout (0x6066) is a fault.
    fn following(&mut self) {
        if self.cia != Cia402::OperationEnabled || self.od.u8(0x6061, 0) as i8 != mode::CSP {
            self.following_since = None;
            return;
        }
        let error = self.od.i32(0x60F4, 0).unsigned_abs();
        if error <= self.od.u32(0x6065, 0) {
            self.following_since = None;
            return;
        }
        let since = *self.following_since.get_or_insert(self.now);
        if self.now - since >= self.following_timeout() {
            self.fault(error_code::FOLLOWING_ERROR);
            self.following_since = None;
        }
    }

    /// Outputs not written for longer than the SM watchdog's time, in
    /// Operational: the drive falls back to Safe-Operational and says why.
    fn watchdog(&mut self) {
        let time = self.esc.sm_watchdog();
        if self.esc.al_state() == state::OP && time > 0.0 && self.now - self.outputs_at > time {
            self.esc.set_al_status(state::SAFE_OP, true, al_code::SM_WATCHDOG);
            self.left_op();
        }
    }

    /// Out of Operational, the outputs no longer reach the drive: it turns
    /// its torque off.
    fn left_op(&mut self) {
        if matches!(
            self.cia,
            Cia402::OperationEnabled | Cia402::QuickStopActive | Cia402::SwitchedOn | Cia402::ReadyToSwitchOn
        ) {
            self.to(Cia402::SwitchOnDisabled);
        }
    }

    /// The master asked for another AL state.
    fn al_control(&mut self) {
        let (asked, ack) = self.esc.al_control();
        let now = self.esc.al_state();
        let error = self.esc.read_u16(super::esc::reg::AL_STATUS) & 0x10 != 0;
        if error && !ack && asked >= now {
            return;
        }
        let refuse = |esc: &mut Esc, code: u16| esc.set_al_status(now, true, code);
        let valid = [state::INIT, state::PRE_OP, state::BOOT, state::SAFE_OP, state::OP];
        if !valid.contains(&asked) {
            return refuse(&mut self.esc, al_code::UNKNOWN_STATE);
        }
        if asked == now {
            self.esc.set_al_status(now, false, 0);
            return;
        }
        match (now, asked) {
            (_, state::INIT) => {
                self.esc.clear_mailboxes();
            }
            (state::INIT, state::PRE_OP) => {
                if let Err(code) = self.check_mailboxes() {
                    return refuse(&mut self.esc, code);
                }
            }
            (state::INIT, state::BOOT) => return refuse(&mut self.esc, al_code::BOOTSTRAP_NOT_SUPPORTED),
            (state::PRE_OP, state::SAFE_OP) => {
                if let Err(code) = self.check_process_data() {
                    return refuse(&mut self.esc, code);
                }
            }
            (state::SAFE_OP, state::OP) => self.outputs_at = self.now,
            (state::SAFE_OP | state::OP, state::PRE_OP) | (state::OP, state::SAFE_OP) => {}
            _ => return refuse(&mut self.esc, al_code::INVALID_STATE_CHANGE),
        }
        if now == state::OP {
            self.left_op();
        }
        self.esc.set_al_status(asked, false, 0);
        self.inputs();
    }

    /// The mailbox sync managers are as the EEPROM says: else, Pre-Op is refused.
    fn check_mailboxes(&self) -> Result<(), u16> {
        let (out, inn) = (self.esc.sync_manager(0), self.esc.sync_manager(1));
        let good = |sm: super::esc::SyncManager, (start, len): (u16, u16), control: u8| {
            sm.enabled && sm.start == start && sm.len == len && sm.control & 0x0F == control & 0x0F
        };
        if good(out, AREAS.mailbox_out, 0x26) && good(inn, AREAS.mailbox_in, 0x22) {
            Ok(())
        } else {
            Err(al_code::INVALID_MAILBOX)
        }
    }

    /// The process data sync managers match the PDOs assigned: else,
    /// Safe-Op is refused.
    fn check_process_data(&self) -> Result<(), u16> {
        let check = |n: usize, bytes: usize, start: u16, written: bool, code: u16| {
            let sm = self.esc.sync_manager(n);
            let ok = if bytes == 0 {
                !sm.enabled || sm.len == 0
            } else {
                sm.enabled
                    && sm.start == start
                    && sm.len as usize == bytes
                    && !sm.mailbox()
                    && sm.written_by_master() == written
            };
            if ok {
                Ok(())
            } else {
                Err(code)
            }
        };
        check(2, self.pdo_bytes(0x1C12), AREAS.outputs.0, true, al_code::INVALID_OUTPUTS)?;
        check(3, self.pdo_bytes(0x1C13), AREAS.inputs.0, false, al_code::INVALID_INPUTS)
    }

    /// The entries of the PDOs assigned in `assign` (0x1C12, 0x1C13), in order.
    fn mapped(&self, assign: u16) -> Vec<(u16, u8, usize)> {
        let mut entries = Vec::new();
        for a in 1..=self.od.u8(assign, 0) {
            let pdo = self.od.u16(assign, a);
            for e in 1..=self.od.u8(pdo, 0) {
                let m = self.od.u32(pdo, e);
                entries.push(((m >> 16) as u16, (m >> 8) as u8, (m & 0xFF) as usize / 8));
            }
        }
        entries
    }

    fn pdo_bytes(&self, assign: u16) -> usize {
        self.mapped(assign).iter().map(|e| e.2).sum()
    }

    /// The inputs' sync manager gets the TxPDOs' values, from Safe-Op on.
    fn inputs(&mut self) {
        self.statusword();
        if self.esc.al_state() < state::SAFE_OP {
            return;
        }
        let mut bytes = Vec::new();
        for (index, sub, n) in self.mapped(0x1C13) {
            match self.od.get(index, sub) {
                Some(v) => bytes.extend_from_slice(&v[..n]),
                None => bytes.extend(std::iter::repeat_n(0, n)),
            }
        }
        let sm = self.esc.sync_manager(3);
        if sm.enabled && sm.len as usize == bytes.len() {
            self.esc.ram_mut(sm.start, bytes.len()).copy_from_slice(&bytes);
        }
    }

    /// New outputs, in Operational: the RxPDOs' values into the dictionary,
    /// then the controlword acts.
    fn outputs(&mut self) {
        self.outputs_at = self.now;
        self.cycled = true;
        if self.esc.al_state() != state::OP {
            return;
        }
        let sm = self.esc.sync_manager(2);
        let data = self.esc.ram(sm.start, sm.len as usize).to_vec();
        let mut at = 0;
        for (index, sub, n) in self.mapped(0x1C12) {
            if let Some(v) = data.get(at..at + n) {
                self.od.set(index, sub, v);
            }
            at += n;
        }
        self.modes();
        self.controlword();
    }

    /// The mode the drive runs (0x6061) follows the one asked (0x6060)
    /// when it's one it has.
    fn modes(&mut self) {
        let asked = self.od.u8(0x6060, 0) as i8;
        if matches!(asked, mode::CSP | mode::CSV | mode::CST) {
            self.od.set(0x6061, 0, &[asked as u8]);
        }
    }

    /// CiA 402's transitions, from the controlword's bits 0 to 3 and 7.
    fn controlword(&mut self) {
        let cw = self.od.u16(0x6040, 0);
        let reset = cw & 0x80 != 0 && self.last_control & 0x80 == 0;
        self.last_control = cw;
        use Cia402::*;
        if reset {
            if self.cia == Fault {
                self.od.set(0x603F, 0, &0u16.to_le_bytes());
                self.to(SwitchOnDisabled);
            }
            return;
        }
        let disable_voltage = cw & 0x02 == 0;
        let quick_stop = !disable_voltage && cw & 0x04 == 0;
        let shutdown = cw & 0x07 == 0x06;
        let switch_on = cw & 0x0F == 0x07;
        let enable = cw & 0x0F == 0x0F;
        let next = match self.cia {
            SwitchOnDisabled if shutdown => ReadyToSwitchOn,
            ReadyToSwitchOn if disable_voltage || quick_stop => SwitchOnDisabled,
            ReadyToSwitchOn if switch_on => SwitchedOn,
            // Switch on and enable at once: transitions 3 then 4.
            ReadyToSwitchOn if enable => OperationEnabled,
            SwitchedOn if disable_voltage || quick_stop => SwitchOnDisabled,
            SwitchedOn if shutdown => ReadyToSwitchOn,
            SwitchedOn if enable => OperationEnabled,
            OperationEnabled if disable_voltage => SwitchOnDisabled,
            OperationEnabled if quick_stop => QuickStopActive,
            OperationEnabled if shutdown => ReadyToSwitchOn,
            OperationEnabled if switch_on => SwitchedOn,
            QuickStopActive if disable_voltage => SwitchOnDisabled,
            s => s,
        };
        if next != self.cia {
            self.to(next);
        }
    }

    fn mailbox(&mut self, message: Vec<u8>) {
        self.esc.free_mailbox();
        if self.esc.al_state() < state::PRE_OP {
            return;
        }
        let preop = self.esc.al_state() == state::PRE_OP;
        let check = |od: &Dictionary, index: u16, sub: u8, v: &[u8]| check_write(od, index, sub, v);
        if let Some(answer) = coe::handle(&message, &mut self.od, preop, &check, &mut self.counter) {
            // The master reads one answer before it sends the next request,
            // so the mailbox to it is free.
            self.esc.post_mailbox(&answer);
        }
    }
}

impl Device for Drive {
    fn station(&self) -> u16 {
        self.esc.station()
    }

    fn datagram(&mut self, reach: Reach, access: Access, data: &mut [u8]) -> u16 {
        self.esc.datagram(reach, access, data)
    }

    fn frame_done(&mut self) {
        let events = std::mem::take(&mut self.esc.events);
        if events.al_control {
            self.al_control();
        }
        if let Some(m) = events.mailbox {
            self.mailbox(m);
        }
        if events.outputs {
            self.outputs();
        }
        self.inputs();
    }
}

/// A write the dictionary's types allow, which the drive checks further:
/// a mapping entry must name a mappable object of its size, in the right
/// direction; an assignment, a mapping object; a count, no more than there are.
fn check_write(od: &Dictionary, index: u16, sub: u8, v: &[u8]) -> Result<(), u32> {
    let rx = (0x1600..0x1800).contains(&index);
    let tx = (0x1A00..0x1C00).contains(&index);
    if (rx || tx || index == 0x1C12 || index == 0x1C13) && sub == 0 {
        let max = if rx || tx { MAP_ENTRIES } else { 1 };
        if v[0] > max {
            return Err(abort::VALUE_TOO_HIGH);
        }
        return Ok(());
    }
    if rx || tx {
        let m = u32::from_le_bytes([v[0], v[1], v[2], v[3]]);
        let (i, s, bits) = ((m >> 16) as u16, (m >> 8) as u8, (m & 0xFF) as usize);
        // Padding: the data types' own indices.
        if (0x0001..=0x0007).contains(&i) {
            return Ok(());
        }
        let e = od.0.get(&(i, s)).ok_or(abort::CANNOT_MAP)?;
        let direction = if rx { e.write == Write::Always } else { e.write == Write::Never };
        if !e.mappable || !direction || e.value.len() * 8 != bits {
            return Err(abort::CANNOT_MAP);
        }
        return Ok(());
    }
    if index == 0x1C12 || index == 0x1C13 {
        let pdo = u16::from_le_bytes([v[0], v[1]]);
        let range = if index == 0x1C12 { 0x1600..0x1800 } else { 0x1A00..0x1C00 };
        if !range.contains(&pdo) || od.get(pdo, 0).is_none() {
            return Err(abort::PDO_TOO_LONG);
        }
    }
    Ok(())
}

fn dictionary(s: &Settings, rx: &Pdo, tx: &Pdo) -> Dictionary {
    use kind::*;
    use Write::*;
    let mut od = Dictionary::default();
    // Device type: profile 402, a servo drive.
    od.add(0x1000, 0, UNSIGNED32, Never, false, &0x0002_0192u32.to_le_bytes());
    od.add(0x1018, 0, UNSIGNED8, Never, false, &[4]);
    for (sub, v) in [(1, s.vendor), (2, s.product), (3, s.revision), (4, s.serial)] {
        od.add(0x1018, sub, UNSIGNED32, Never, false, &v.to_le_bytes());
    }
    for pdo in [rx, tx] {
        od.add(pdo.index, 0, UNSIGNED8, PreOp, false, &[pdo.entries.len() as u8]);
        for sub in 1..=MAP_ENTRIES {
            let value = pdo
                .entries
                .get(sub as usize - 1)
                .map_or(0, |&(i, s, bits, _)| ((i as u32) << 16) | ((s as u32) << 8) | bits as u32);
            od.add(pdo.index, sub, UNSIGNED32, PreOp, false, &value.to_le_bytes());
        }
    }
    // The sync managers' communication types: mailbox out and in, outputs, inputs.
    od.add(0x1C00, 0, UNSIGNED8, Never, false, &[4]);
    for (sub, t) in [(1, 1u8), (2, 2), (3, 3), (4, 4)] {
        od.add(0x1C00, sub, UNSIGNED8, Never, false, &[t]);
    }
    od.add(0x1C12, 0, UNSIGNED8, PreOp, false, &[1]);
    od.add(0x1C12, 1, UNSIGNED16, PreOp, false, &rx.index.to_le_bytes());
    od.add(0x1C13, 0, UNSIGNED8, PreOp, false, &[1]);
    od.add(0x1C13, 1, UNSIGNED16, PreOp, false, &tx.index.to_le_bytes());

    od.add(0x603F, 0, UNSIGNED16, Never, true, &[0, 0]);
    od.add(0x6040, 0, UNSIGNED16, Always, true, &[0, 0]);
    od.add(0x6041, 0, UNSIGNED16, Never, true, &[0, 0]);
    // Quick stop option code 2: slow down, then Switch On Disabled.
    od.add(0x605A, 0, INTEGER16, Never, false, &2i16.to_le_bytes());
    // No mode until the master sets one, as drives leave the factory.
    od.add(0x6060, 0, INTEGER8, Always, true, &[0]);
    od.add(0x6061, 0, INTEGER8, Never, true, &[0]);
    od.add(0x6064, 0, INTEGER32, Never, true, &[0; 4]);
    // drydock's defaults: a following error of a turn, for 10 ms.
    od.add(0x6065, 0, UNSIGNED32, Always, false, &s.counts_per_turn.to_le_bytes());
    od.add(0x6066, 0, UNSIGNED16, Always, false, &10u16.to_le_bytes());
    od.add(0x606C, 0, INTEGER32, Never, true, &[0; 4]);
    od.add(0x6071, 0, INTEGER16, Always, true, &[0; 2]);
    let max = (s.peak_torque / s.rated_torque * 1000.0).round().min(u16::MAX as f64) as u16;
    od.add(0x6072, 0, UNSIGNED16, Always, true, &max.to_le_bytes());
    od.add(0x6076, 0, UNSIGNED32, Never, false, &((s.rated_torque * 1000.0).round() as u32).to_le_bytes());
    od.add(0x6077, 0, INTEGER16, Never, true, &[0; 2]);
    od.add(0x607A, 0, INTEGER32, Always, true, &[0; 4]);
    od.add(0x608F, 0, UNSIGNED8, Never, false, &[2]);
    od.add(0x608F, 1, UNSIGNED32, Never, false, &s.counts_per_turn.to_le_bytes());
    od.add(0x608F, 2, UNSIGNED32, Never, false, &1u32.to_le_bytes());
    od.add(0x60F4, 0, INTEGER32, Never, true, &[0; 4]);
    od.add(0x60FF, 0, INTEGER32, Always, true, &[0; 4]);
    // Supported modes: CSP, CSV, CST (bits 7, 8, 9).
    od.add(0x6502, 0, UNSIGNED32, Never, false, &0x0380u32.to_le_bytes());
    od
}
