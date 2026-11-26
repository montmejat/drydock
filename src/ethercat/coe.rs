//! CANopen over EtherCAT (CoE): the mailbox's SDO transfers, which read and
//! write the drive's object dictionary while it's configured (from
//! Pre-Operational on).
//!
//! A mailbox message is a 6-byte header (length, address, channel, type
//! and counter) and its data; CoE's data is a 2-byte header (service: 2, an
//! SDO request; 3, its response) then the SDO's 8 bytes (command, index,
//! subindex, 4 bytes of data or the size), then any more data.
//!
//! Sources: ETG.1000.6 by way of the CANopen SDO protocol (CiA 301) it
//! carries, the transfers SOEM and ethercrab make, and their abort codes.
//! Supported: expedited and normal uploads and downloads, and complete
//! access. Not supported: segmented transfers (longer than a mailbox), SDO
//! information, emergencies.

use std::collections::BTreeMap;

/// Mailbox types (the header's low nibble).
const MAILBOX_ERROR: u8 = 0x00;
const MAILBOX_COE: u8 = 0x03;
/// CoE services.
const SDO_REQUEST: u16 = 2;
const SDO_RESPONSE: u16 = 3;

/// SDO abort codes (CiA 301).
pub mod abort {
    pub const COMMAND: u32 = 0x0504_0001;
    pub const READ_ONLY: u32 = 0x0601_0002;
    pub const NO_OBJECT: u32 = 0x0602_0000;
    pub const CANNOT_MAP: u32 = 0x0604_0041;
    pub const PDO_TOO_LONG: u32 = 0x0604_0042;
    pub const LENGTH: u32 = 0x0607_0010;
    pub const NO_SUBINDEX: u32 = 0x0609_0011;
    pub const VALUE_TOO_HIGH: u32 = 0x0609_0031;
    pub const STATE: u32 = 0x0800_0022;
}

/// CoE data types, by their codes (CiA 301).
pub mod kind {
    pub const INTEGER8: u16 = 0x0002;
    pub const INTEGER16: u16 = 0x0003;
    pub const INTEGER32: u16 = 0x0004;
    pub const UNSIGNED8: u16 = 0x0005;
    pub const UNSIGNED16: u16 = 0x0006;
    pub const UNSIGNED32: u16 = 0x0007;
}

/// Who may write an entry, and when.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Write {
    Never,
    /// In Pre-Operational only: PDO mappings and assignments.
    PreOp,
    Always,
}

#[derive(Debug, Clone)]
pub struct Entry {
    pub value: Vec<u8>,
    pub kind: u16,
    pub write: Write,
    /// It may be mapped into a PDO: RxPDO if written by the master.
    pub mappable: bool,
}

/// A device's object dictionary: (index, subindex) to entry. An array's or
/// a record's subindex 0 is its count of entries.
#[derive(Debug, Clone, Default)]
pub struct Dictionary(pub BTreeMap<(u16, u8), Entry>);

impl Dictionary {
    pub fn add(&mut self, index: u16, sub: u8, kind: u16, write: Write, mappable: bool, value: &[u8]) {
        self.0.insert((index, sub), Entry { value: value.to_vec(), kind, write, mappable });
    }

    pub fn get(&self, index: u16, sub: u8) -> Option<&[u8]> {
        self.0.get(&(index, sub)).map(|e| e.value.as_slice())
    }

    /// Sets a value the firmware owns (an actual position), whatever who may write it.
    pub fn set(&mut self, index: u16, sub: u8, value: &[u8]) {
        if let Some(e) = self.0.get_mut(&(index, sub)) {
            let n = e.value.len().min(value.len());
            e.value[..n].copy_from_slice(&value[..n]);
        }
    }

    pub fn u8(&self, index: u16, sub: u8) -> u8 {
        self.get(index, sub).map_or(0, |v| v[0])
    }
    pub fn u16(&self, index: u16, sub: u8) -> u16 {
        self.get(index, sub).map_or(0, |v| u16::from_le_bytes([v[0], v[1]]))
    }
    pub fn u32(&self, index: u16, sub: u8) -> u32 {
        self.get(index, sub).map_or(0, |v| u32::from_le_bytes([v[0], v[1], v[2], v[3]]))
    }
    pub fn i32(&self, index: u16, sub: u8) -> i32 {
        self.u32(index, sub) as i32
    }

    fn has_index(&self, index: u16) -> bool {
        self.0.range((index, 0)..=(index, 255)).next().is_some()
    }

    /// Subindices 1 to the count in subindex 0, for complete access.
    fn subs(&self, index: u16) -> Vec<u8> {
        (1..=self.u8(index, 0)).filter(|&s| self.0.contains_key(&(index, s))).collect()
    }
}

/// A check the firmware makes on a write before it's done: an entry in a
/// PDO mapping must name a mappable object of its length.
pub type Check<'a> = &'a dyn Fn(&Dictionary, u16, u8, &[u8]) -> Result<(), u32>;

/// A mailbox message from the master: the answer to post, if any.
/// `preop`: the device is in Pre-Operational (PDO configuration may change).
pub fn handle(message: &[u8], od: &mut Dictionary, preop: bool, check: Check, counter: &mut u8) -> Option<Vec<u8>> {
    if message.len() < 6 {
        return None;
    }
    let len = u16::from_le_bytes([message[0], message[1]]) as usize;
    let kind = message[5] & 0x0F;
    if kind != MAILBOX_COE {
        // A mailbox error: the protocol isn't supported (ETG.1000.4).
        return Some(reply(MAILBOX_ERROR, counter, &[0x01, 0x00, 0x02, 0x00]));
    }
    let data = &message[6..(6 + len).min(message.len())];
    if data.len() < 10 {
        return None;
    }
    let service = u16::from_le_bytes([data[0], data[1]]) >> 12;
    if service != SDO_REQUEST {
        return Some(reply(MAILBOX_ERROR, counter, &[0x01, 0x00, 0x02, 0x00]));
    }
    let sdo = &data[2..];
    let (command, index, sub) = (sdo[0], u16::from_le_bytes([sdo[1], sdo[2]]), sdo[3]);
    let complete = command & 0x10 != 0;
    let answer = match command >> 5 {
        2 => upload(od, index, sub, complete),
        1 => download(od, index, sub, complete, command, sdo, preop, check),
        _ => Err(abort::COMMAND),
    };
    let mut coe = (SDO_RESPONSE << 12).to_le_bytes().to_vec();
    match answer {
        Ok(body) => coe.extend(body),
        Err(code) => {
            coe.push(0x80);
            coe.extend_from_slice(&index.to_le_bytes());
            coe.push(sub);
            coe.extend_from_slice(&code.to_le_bytes());
        }
    }
    Some(reply(MAILBOX_COE, counter, &coe))
}

/// A mailbox message from the device: its header, then `data`. The
/// counter goes 1 to 7, then round again.
fn reply(kind: u8, counter: &mut u8, data: &[u8]) -> Vec<u8> {
    *counter = if *counter >= 7 { 1 } else { *counter + 1 };
    let mut m = (data.len() as u16).to_le_bytes().to_vec();
    m.extend_from_slice(&[0, 0, 0, kind | (*counter << 4)]);
    m.extend_from_slice(data);
    m
}

fn missing(od: &Dictionary, index: u16) -> u32 {
    if od.has_index(index) {
        abort::NO_SUBINDEX
    } else {
        abort::NO_OBJECT
    }
}

fn upload(od: &Dictionary, index: u16, sub: u8, complete: bool) -> Result<Vec<u8>, u32> {
    let value = if complete {
        if sub > 1 || !od.has_index(index) {
            return Err(missing(od, index));
        }
        // Subindex 0 counts as 16 bits in a complete access.
        let mut v = if sub == 0 { vec![od.u8(index, 0), 0] } else { Vec::new() };
        for s in od.subs(index) {
            v.extend_from_slice(od.get(index, s).unwrap());
        }
        v
    } else {
        od.get(index, sub).ok_or_else(|| missing(od, index))?.to_vec()
    };
    let mut b = Vec::new();
    if value.len() <= 4 && !complete {
        // Expedited: the data in the SDO itself, with how many of its 4
        // bytes are unused.
        b.push(0x43 | (((4 - value.len()) as u8) << 2));
        b.extend_from_slice(&index.to_le_bytes());
        b.push(sub);
        let mut four = value.clone();
        four.resize(4, 0);
        b.extend(four);
    } else {
        b.push(0x41 | if complete { 0x10 } else { 0 });
        b.extend_from_slice(&index.to_le_bytes());
        b.push(sub);
        b.extend_from_slice(&(value.len() as u32).to_le_bytes());
        b.extend(value);
    }
    Ok(b)
}

#[allow(clippy::too_many_arguments)]
fn download(
    od: &mut Dictionary,
    index: u16,
    sub: u8,
    complete: bool,
    command: u8,
    sdo: &[u8],
    preop: bool,
    check: Check,
) -> Result<Vec<u8>, u32> {
    let loose = command & 0x03 == 0x02;
    let data: Vec<u8> = if command & 0x02 != 0 {
        let unused = if command & 0x01 != 0 { ((command >> 2) & 0x03) as usize } else { 0 };
        sdo[4..8][..4 - unused].to_vec()
    } else {
        let size = u32::from_le_bytes([sdo[4], sdo[5], sdo[6], sdo[7]]) as usize;
        sdo.get(8..8 + size).ok_or(abort::LENGTH)?.to_vec()
    };
    if complete {
        if sub > 1 || !od.has_index(index) {
            return Err(missing(od, index));
        }
        let mut rest = &data[..];
        let count = if sub == 0 {
            let c = *rest.first().ok_or(abort::LENGTH)?;
            rest = rest.get(2..).ok_or(abort::LENGTH)?;
            Some(c)
        } else {
            None
        };
        let mut s = 1u8;
        while !rest.is_empty() {
            let n = od.0.get(&(index, s)).ok_or(abort::NO_SUBINDEX)?.value.len();
            let v = rest.get(..n).ok_or(abort::LENGTH)?;
            write(od, index, s, v, false, preop, check)?;
            rest = &rest[n..];
            s += 1;
        }
        if let Some(c) = count {
            write(od, index, 0, &[c], false, preop, check)?;
        }
    } else {
        write(od, index, sub, &data, loose, preop, check)?;
    }
    let mut b = vec![0x60 | if complete { 0x10 } else { 0 }];
    b.extend_from_slice(&index.to_le_bytes());
    b.push(sub);
    b.extend_from_slice(&[0; 4]);
    Ok(b)
}

fn write(
    od: &mut Dictionary,
    index: u16,
    sub: u8,
    data: &[u8],
    loose: bool,
    preop: bool,
    check: Check,
) -> Result<(), u32> {
    let entry = od.0.get(&(index, sub)).ok_or_else(|| missing(od, index))?;
    match entry.write {
        Write::Never => return Err(abort::READ_ONLY),
        Write::PreOp if !preop => return Err(abort::STATE),
        _ => {}
    }
    // An expedited write that doesn't say its size carries 4 bytes, of
    // which the entry takes its own length (CiA 301); any other must match.
    let n = entry.value.len();
    if data.len() != n && !(loose && data.len() == 4 && n < 4) {
        return Err(abort::LENGTH);
    }
    let mut v = data.to_vec();
    v.truncate(n);
    check(od, index, sub, &v)?;
    od.0.get_mut(&(index, sub)).unwrap().value = v;
    Ok(())
}
