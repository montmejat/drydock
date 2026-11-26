//! The chip: a Beckhoff ET1100 EtherCAT Slave Controller (ESC), as the
//! master sees it over the wire. It's memory: registers from 0x0000 to
//! 0x0FFF, process data RAM from 0x1000 to 0x2FFF (8 KB). The master reads
//! and writes it with datagrams; the drive's firmware sits on the other side
//! (the PDI) and reacts to what was written.
//!
//! Source: Beckhoff, "EtherCAT Slave Controller, Section II, Register
//! Description" (the register map, the EEPROM interface, sync managers and
//! FMMUs) and "Section III, ET1100" (its counts of FMMUs, sync managers and
//! RAM). Where drydock's chip differs from an ET1100: it has no distributed
//! clocks (its feature register says so, so masters don't use them), its
//! FMMUs map whole bytes only, and a sync manager's three buffers are one.

use super::frame::{Access, Reach};

/// Register addresses (Section II).
pub mod reg {
    pub const TYPE: u16 = 0x0000;
    pub const REVISION: u16 = 0x0001;
    pub const BUILD: u16 = 0x0002;
    pub const FMMUS: u16 = 0x0004;
    pub const SYNC_MANAGERS: u16 = 0x0005;
    pub const RAM_SIZE: u16 = 0x0006;
    pub const PORTS: u16 = 0x0007;
    pub const FEATURES: u16 = 0x0008;
    pub const STATION: u16 = 0x0010;
    pub const ALIAS: u16 = 0x0012;
    pub const DL_STATUS: u16 = 0x0110;
    pub const AL_CONTROL: u16 = 0x0120;
    pub const AL_STATUS: u16 = 0x0130;
    pub const AL_STATUS_CODE: u16 = 0x0134;
    pub const PDI_CONTROL: u16 = 0x0140;
    pub const WATCHDOG_DIVIDER: u16 = 0x0400;
    pub const SM_WATCHDOG: u16 = 0x0420;
    pub const SM_WATCHDOG_STATUS: u16 = 0x0440;
    pub const EEPROM_CONFIG: u16 = 0x0500;
    pub const EEPROM_CONTROL: u16 = 0x0502;
    pub const EEPROM_ADDRESS: u16 = 0x0504;
    pub const EEPROM_DATA: u16 = 0x0508;
    pub const FMMU: u16 = 0x0600;
    pub const SM: u16 = 0x0800;
}

/// An ET1100's: 8 FMMUs, 8 sync managers, 8 KB of process RAM.
pub const FMMU_COUNT: usize = 8;
pub const SM_COUNT: usize = 8;
const MEMORY: usize = 0x3000;

/// AL states (register 0x0130's low nibble).
pub mod state {
    pub const INIT: u8 = 1;
    pub const PRE_OP: u8 = 2;
    pub const BOOT: u8 = 3;
    pub const SAFE_OP: u8 = 4;
    pub const OP: u8 = 8;
}

/// A sync manager's settings, as the master wrote them (0x0800 + 8 n).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncManager {
    pub start: u16,
    pub len: u16,
    pub control: u8,
    pub enabled: bool,
}

impl SyncManager {
    pub fn mailbox(&self) -> bool {
        self.control & 0x03 == 0x02
    }
    /// The master writes it (outputs, the mailbox to the device).
    pub fn written_by_master(&self) -> bool {
        self.control & 0x0C == 0x04
    }
    fn covers(&self, addr: usize) -> bool {
        self.enabled && self.len > 0 && (self.start as usize..self.start as usize + self.len as usize).contains(&addr)
    }
    fn last(&self) -> usize {
        self.start as usize + self.len as usize - 1
    }
}

pub struct Esc {
    mem: Vec<u8>,
    eeprom: Vec<u8>,
    /// Each mailbox sync manager's buffer: full or empty.
    full: [bool; SM_COUNT],
    /// What the firmware has to act on, once the frame is through.
    pub events: Events,
}

/// What the master did that the firmware hears of (the chip's AL event
/// register, 0x0220, in short).
#[derive(Debug, Default, Clone)]
pub struct Events {
    /// The master wrote AL Control.
    pub al_control: bool,
    /// A mailbox the master wrote in full, from sync manager 0 (its data).
    pub mailbox: Option<Vec<u8>>,
    /// The master wrote the outputs' sync manager in full.
    pub outputs: bool,
    /// The master read the device's mailbox in full: it's free again.
    pub mailbox_read: bool,
}

impl Esc {
    /// A chip whose EEPROM holds `eeprom`, as it is after power-on: its
    /// station alias loaded from it, in Init.
    pub fn new(eeprom: Vec<u8>) -> Self {
        let mut esc = Self { mem: vec![0; MEMORY], eeprom, full: [false; SM_COUNT], events: Events::default() };
        let m = &mut esc.mem;
        m[reg::TYPE as usize] = 0x11; // ET1100
        m[reg::REVISION as usize] = 0x00;
        m[reg::BUILD as usize..][..2].copy_from_slice(&0x0002u16.to_le_bytes());
        m[reg::FMMUS as usize] = FMMU_COUNT as u8;
        m[reg::SYNC_MANAGERS as usize] = SM_COUNT as u8;
        m[reg::RAM_SIZE as usize] = 8;
        // Ports 0 and 1 are MII (0b11 each), 2 and 3 not implemented.
        m[reg::PORTS as usize] = 0x0F;
        // An ET1100's features (0x01FC) less distributed clocks (bits 2, 3
        // and 8): drydock's chip has none.
        m[reg::FEATURES as usize..][..2].copy_from_slice(&0x00F0u16.to_le_bytes());
        // The watchdog's defaults: a 100 µs tick, 100 ms for the outputs.
        m[reg::WATCHDOG_DIVIDER as usize..][..2].copy_from_slice(&0x09C2u16.to_le_bytes());
        m[reg::SM_WATCHDOG as usize..][..2].copy_from_slice(&0x03E8u16.to_le_bytes());
        m[reg::SM_WATCHDOG_STATUS as usize] = 0x01;
        // From the EEPROM at power-on: the PDI's settings and the alias.
        let word = |w: usize| u16::from_le_bytes([esc.eeprom[2 * w], esc.eeprom[2 * w + 1]]);
        let (pdi, alias) = (word(0), word(4));
        let m = &mut esc.mem;
        m[reg::PDI_CONTROL as usize..][..2].copy_from_slice(&pdi.to_le_bytes());
        m[reg::ALIAS as usize..][..2].copy_from_slice(&alias.to_le_bytes());
        // EEPROM: reads 8 bytes at a time (bit 6), as an ET1100 does.
        m[reg::EEPROM_CONTROL as usize] = 0x40;
        esc.set_al_status(state::INIT, false, 0);
        esc.set_line(false);
        esc
    }

    /// Whether another device is plugged into port 1: the DL status the
    /// master reads to learn the line's shape (0x0110).
    pub fn set_line(&mut self, next: bool) {
        // PDI operational, its watchdog not expired; port 0 linked and
        // carrying the frames; ports 2 and 3 closed.
        let mut status: u16 = 0x0001 | 0x0002 | 0x0010 | 0x0200 | 0x1000 | 0x4000;
        status |= if next { 0x0020 | 0x0800 } else { 0x0400 };
        self.write_u16(reg::DL_STATUS, status);
    }

    pub fn station(&self) -> u16 {
        self.read_u16(reg::STATION)
    }

    pub fn read_u16(&self, addr: u16) -> u16 {
        u16::from_le_bytes([self.mem[addr as usize], self.mem[addr as usize + 1]])
    }

    fn write_u16(&mut self, addr: u16, value: u16) {
        self.mem[addr as usize..][..2].copy_from_slice(&value.to_le_bytes());
    }

    /// What the master asked for in AL Control: the state, and whether it
    /// acknowledges an error (bit 4).
    pub fn al_control(&self) -> (u8, bool) {
        let c = self.read_u16(reg::AL_CONTROL);
        ((c & 0x0F) as u8, c & 0x10 != 0)
    }

    /// The firmware sets the state, the error flag and its code.
    pub fn set_al_status(&mut self, state: u8, error: bool, code: u16) {
        self.write_u16(reg::AL_STATUS, state as u16 | if error { 0x10 } else { 0 });
        self.write_u16(reg::AL_STATUS_CODE, code);
    }

    pub fn al_state(&self) -> u8 {
        self.mem[reg::AL_STATUS as usize] & 0x0F
    }

    pub fn sync_manager(&self, n: usize) -> SyncManager {
        let at = reg::SM as usize + 8 * n;
        let m = &self.mem;
        SyncManager {
            start: u16::from_le_bytes([m[at], m[at + 1]]),
            len: u16::from_le_bytes([m[at + 2], m[at + 3]]),
            control: m[at + 4],
            enabled: m[at + 6] & 0x01 != 0,
        }
    }

    /// The SM watchdog's time, seconds: its count of the divider's ticks
    /// (each (divider + 2) / 25 MHz). Zero is off.
    pub fn sm_watchdog(&self) -> f64 {
        let tick = (self.read_u16(reg::WATCHDOG_DIVIDER) as f64 + 2.0) / 25e6;
        self.read_u16(reg::SM_WATCHDOG) as f64 * tick
    }

    /// The firmware's side of process data RAM.
    pub fn ram(&self, start: u16, len: usize) -> &[u8] {
        &self.mem[start as usize..start as usize + len]
    }

    pub fn ram_mut(&mut self, start: u16, len: usize) -> &mut [u8] {
        &mut self.mem[start as usize..start as usize + len]
    }

    /// The firmware puts its answer in the mailbox to the master (sync
    /// manager 1), which then reads as full. False if it's still full.
    pub fn post_mailbox(&mut self, data: &[u8]) -> bool {
        let sm = self.sync_manager(1);
        if self.full[1] || !sm.enabled || data.len() > sm.len as usize {
            return false;
        }
        let area = self.ram_mut(sm.start, sm.len as usize);
        area.fill(0);
        area[..data.len()].copy_from_slice(data);
        self.full[1] = true;
        true
    }

    /// The firmware took the master's mailbox (sync manager 0): it's free
    /// for the next one.
    pub fn free_mailbox(&mut self) {
        self.full[0] = false;
    }

    /// The firmware leaves Init: what the mailboxes held goes.
    pub fn clear_mailboxes(&mut self) {
        self.full = [false; SM_COUNT];
    }

    /// Sync managers' status registers (0x0805 + 8 n), as they read now:
    /// bit 3, a mailbox is full.
    fn refresh_status(&mut self) {
        for n in 0..SM_COUNT {
            self.mem[reg::SM as usize + 8 * n + 5] = if self.full[n] { 0x08 } else { 0x00 };
        }
    }

    /// The registers the master may write; the rest are read-only to it.
    fn writable(addr: usize) -> bool {
        match addr {
            0x0010..=0x0011 | 0x0100..=0x0103 | 0x0108..=0x0109 | 0x0120..=0x0121 | 0x0200..=0x0201 => true,
            // Error counters: writing clears them.
            0x0300..=0x0313 => true,
            0x0400..=0x0401 | 0x0410..=0x0411 | 0x0420..=0x0421 => true,
            0x0500 | 0x0502..=0x050F => true,
            0x0600..=0x067F => true,
            // A sync manager's status (byte 5) and PDI control (byte 7) are the PDI's.
            0x0800..=0x083F => !matches!((addr - 0x0800) % 8, 5 | 7),
            // User RAM, then process data RAM.
            0x0F80..=0x2FFF => true,
            _ => false,
        }
    }

    /// A datagram's access to this chip: how much the working counter goes up.
    pub fn datagram(&mut self, reach: Reach, access: Access, data: &mut [u8]) -> u16 {
        self.refresh_status();
        let wkc = match reach {
            Reach::None => 0,
            Reach::Here { ado } => self.physical(ado as usize, access, data, false),
            Reach::All { ado } => self.physical(ado as usize, access, data, true),
            Reach::Logical { address } => self.logical(address, access, data),
        };
        self.after_writes();
        wkc
    }

    /// The mailbox sync manager whose area [addr, addr + len) touches, if any.
    fn mailbox_in(&self, addr: usize, len: usize) -> Option<usize> {
        (0..SM_COUNT).find(|&n| {
            let sm = self.sync_manager(n);
            sm.mailbox() && (addr..addr + len).any(|a| sm.covers(a))
        })
    }

    fn physical(&mut self, addr: usize, access: Access, data: &mut [u8], broadcast: bool) -> u16 {
        let len = data.len();
        if let Some(n) = self.mailbox_in(addr, len) {
            // A mailbox is read only when full, written only when empty;
            // otherwise the access fails and the counter stays.
            let sm = self.sync_manager(n);
            let ok = if sm.written_by_master() { access.write && !self.full[n] } else { access.read && self.full[n] };
            if !ok {
                return 0;
            }
        }
        let mut wkc = 0;
        let old: Vec<u8> = (0..len).map(|i| self.byte(addr + i)).collect();
        if access.write {
            for (i, &b) in data.iter().enumerate() {
                self.write_byte(addr + i, b);
            }
            self.written(addr, len);
            wkc += if access.read { 2 } else { 1 };
        }
        if access.read {
            for (i, d) in data.iter_mut().enumerate() {
                // A broadcast read ORs every device's value into the data.
                *d = if broadcast { *d | old[i] } else { old[i] };
            }
            self.read(addr, len);
            wkc += 1;
        }
        wkc
    }

    fn logical(&mut self, address: u32, access: Access, data: &mut [u8]) -> u16 {
        let (mut read, mut wrote) = (false, false);
        let start = address as u64;
        let end = start + data.len() as u64;
        for f in 0..FMMU_COUNT {
            let at = reg::FMMU as usize + 16 * f;
            let m = &self.mem;
            if m[at + 12] & 0x01 == 0 {
                continue;
            }
            let log = u32::from_le_bytes([m[at], m[at + 1], m[at + 2], m[at + 3]]) as u64;
            let len = u16::from_le_bytes([m[at + 4], m[at + 5]]) as u64;
            let phys = u16::from_le_bytes([m[at + 8], m[at + 9]]) as usize;
            let kind = m[at + 11];
            let (lo, hi) = (start.max(log), end.min(log + len));
            if lo >= hi {
                continue;
            }
            let (from, to) = ((lo - start) as usize, (hi - start) as usize);
            let p = phys + (lo - log) as usize;
            let n = to - from;
            if kind & 0x02 != 0 && access.write {
                for i in 0..n {
                    self.write_byte(p + i, data[from + i]);
                }
                self.written(p, n);
                wrote = true;
            }
            if kind & 0x01 != 0 && access.read {
                for i in 0..n {
                    data[from + i] = self.byte(p + i);
                }
                self.read(p, n);
                read = true;
            }
        }
        // LRW: a read counts one, a write two.
        read as u16
            + if wrote {
                if access.read {
                    2
                } else {
                    1
                }
            } else {
                0
            }
    }

    fn byte(&self, addr: usize) -> u8 {
        self.mem.get(addr).copied().unwrap_or(0)
    }

    fn write_byte(&mut self, addr: usize, b: u8) {
        if Self::writable(addr) {
            self.mem[addr] = b;
            if addr == reg::AL_CONTROL as usize || addr == reg::AL_CONTROL as usize + 1 {
                self.events.al_control = true;
            }
        }
    }

    /// The master wrote [addr, addr + len): a sync manager it fills, when
    /// its last byte is written, hands its buffer to the firmware.
    fn written(&mut self, addr: usize, len: usize) {
        for n in 0..SM_COUNT {
            let sm = self.sync_manager(n);
            if !sm.covers(addr) && !(addr..addr + len).any(|a| sm.covers(a)) {
                continue;
            }
            if sm.written_by_master() && (addr..addr + len).contains(&sm.last()) {
                if sm.mailbox() {
                    self.full[n] = true;
                    self.events.mailbox = Some(self.ram(sm.start, sm.len as usize).to_vec());
                } else {
                    self.events.outputs = true;
                }
            }
        }
    }

    /// The master read [addr, addr + len): a full mailbox whose last byte
    /// it read is empty again.
    fn read(&mut self, addr: usize, len: usize) {
        for n in 0..SM_COUNT {
            let sm = self.sync_manager(n);
            if sm.mailbox() && !sm.written_by_master() && self.full[n] && (addr..addr + len).contains(&sm.last()) {
                self.full[n] = false;
                self.events.mailbox_read = true;
            }
        }
    }

    /// The EEPROM interface acts on a command the master wrote (bits 8 to
    /// 10 of 0x0502): read 8 bytes, write a word, or reload. It's done at
    /// once: the busy bit (15) never shows.
    fn after_writes(&mut self) {
        let control = self.read_u16(reg::EEPROM_CONTROL);
        let command = (control >> 8) & 0x07;
        if command == 0 {
            return;
        }
        let address = u32::from_le_bytes(self.mem[0x0504..0x0508].try_into().unwrap()) as usize * 2;
        let mut error = false;
        match command {
            0b001 => {
                for i in 0..8 {
                    self.mem[reg::EEPROM_DATA as usize + i] = self.eeprom.get(address + i).copied().unwrap_or(0xFF);
                }
            }
            0b010 => {
                if control & 0x0001 != 0 && address + 1 < self.eeprom.len() {
                    self.eeprom[address] = self.mem[reg::EEPROM_DATA as usize];
                    self.eeprom[address + 1] = self.mem[reg::EEPROM_DATA as usize + 1];
                } else {
                    error = true;
                }
            }
            0b100 => {}
            _ => error = true,
        }
        // The command bits clear when it's done; bit 13 says it failed. Bit
        // 6 (8-byte reads) is the chip's, whatever the master wrote there.
        let status = (control & 0x0081) | 0x0040 | if error { 0x2000 } else { 0 };
        self.write_u16(reg::EEPROM_CONTROL, status);
    }
}
