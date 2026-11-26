//! An EtherCAT frame, and the line of devices it passes through.
//!
//! On the wire: an Ethernet header (EtherType 0x88A4), a 2-byte EtherCAT
//! header (length, type 1: datagrams), then datagrams back to back. Each
//! datagram is a 10-byte header (command, index, address, length and
//! "more follow", IRQ), its data, and a 2-byte working counter that each
//! device it reaches adds to. The master compares that count with what it
//! expected: one short means a device didn't answer.
//!
//! The ESC datasheet (Beckhoff, "EtherCAT Slave Controller, Section I",
//! frame processing) is the source for the commands and the counter.

/// The datagram commands (ESC datasheet, Section I, table "EtherCAT
/// command types").
pub mod cmd {
    pub const NOP: u8 = 0;
    pub const APRD: u8 = 1;
    pub const APWR: u8 = 2;
    pub const APRW: u8 = 3;
    pub const FPRD: u8 = 4;
    pub const FPWR: u8 = 5;
    pub const FPRW: u8 = 6;
    pub const BRD: u8 = 7;
    pub const BWR: u8 = 8;
    pub const BRW: u8 = 9;
    pub const LRD: u8 = 10;
    pub const LWR: u8 = 11;
    pub const LRW: u8 = 12;
    pub const ARMW: u8 = 13;
    pub const FRMW: u8 = 14;
}

/// Ethernet header, then EtherCAT's.
const ETH_HEADER: usize = 14;
const ECAT_HEADER: usize = 2;
const DATAGRAM_HEADER: usize = 10;

/// How a datagram reaches one device: which device it addresses, and how.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reach {
    /// Not this device.
    None,
    /// This device, at physical address `ado`.
    Here { ado: u16 },
    /// Every device, at `ado`.
    All { ado: u16 },
    /// Logical addresses, through each device's FMMUs.
    Logical { address: u32 },
}

/// What a datagram does: read, write or both.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Access {
    pub read: bool,
    pub write: bool,
}

pub fn access(command: u8) -> Access {
    use cmd::*;
    match command {
        APRD | FPRD | BRD | LRD => Access { read: true, write: false },
        APWR | FPWR | BWR | LWR => Access { read: false, write: true },
        APRW | FPRW | BRW | LRW => Access { read: true, write: true },
        _ => Access { read: false, write: false },
    }
}

/// One device's view of the frames passing through it.
pub trait Device {
    /// Its configured station address (register 0x0010).
    fn station(&self) -> u16;
    /// A datagram reached it, as `reach` says, with `access`. It reads into
    /// or writes from `data` and says how much the working counter goes up.
    fn datagram(&mut self, reach: Reach, access: Access, data: &mut [u8]) -> u16;
    /// A frame is over: what the chip passed on to the firmware acts now.
    fn frame_done(&mut self);
}

/// Why a frame went no further than drydock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dropped {
    /// Shorter than its headers say, or not datagrams.
    Malformed,
}

/// Passes a frame through `devices`, in order, as down a cable and back.
/// `reachable` is how many of them the frame gets to: a device whose link is
/// down cuts the line there, and the frame turns at the one before it.
/// The frame is changed in place, as the chips change it.
pub fn pass(frame: &mut [u8], devices: &mut [&mut dyn Device]) -> Result<(), Dropped> {
    if frame.len() < ETH_HEADER + ECAT_HEADER || u16::from_be_bytes([frame[12], frame[13]]) != super::ETHERTYPE {
        return Err(Dropped::Malformed);
    }
    let header = u16::from_le_bytes([frame[14], frame[15]]);
    let (len, kind) = ((header & 0x07FF) as usize, header >> 12);
    let body = ETH_HEADER + ECAT_HEADER;
    if kind != 1 || frame.len() < body + len {
        return Err(Dropped::Malformed);
    }
    // Datagrams' places in the frame, checked before any device sees one,
    // as a chip checks the frame's length before it acts on its writes.
    let mut places = Vec::new();
    let mut at = body;
    loop {
        if at + DATAGRAM_HEADER + 2 > body + len {
            return Err(Dropped::Malformed);
        }
        let dlen = (u16::from_le_bytes([frame[at + 6], frame[at + 7]]) & 0x07FF) as usize;
        let more = frame[at + 7] & 0x80 != 0;
        if at + DATAGRAM_HEADER + dlen + 2 > body + len {
            return Err(Dropped::Malformed);
        }
        places.push((at, dlen));
        at += DATAGRAM_HEADER + dlen + 2;
        if !more {
            break;
        }
    }
    // Each device processes the whole frame in turn: the first device's
    // changes are what the second one sees, as on a cable.
    for device in devices.iter_mut() {
        for &(at, dlen) in &places {
            let command = frame[at];
            let adp = u16::from_le_bytes([frame[at + 2], frame[at + 3]]);
            let ado = u16::from_le_bytes([frame[at + 4], frame[at + 5]]);
            let logical = u32::from_le_bytes([frame[at + 2], frame[at + 3], frame[at + 4], frame[at + 5]]);
            let reach = match command {
                cmd::APRD | cmd::APWR | cmd::APRW | cmd::ARMW => {
                    // Auto-increment: each device adds one to the position as
                    // the datagram passes; the one that gets it at zero is it.
                    frame[at + 2..at + 4].copy_from_slice(&adp.wrapping_add(1).to_le_bytes());
                    if adp == 0 {
                        Reach::Here { ado }
                    } else {
                        Reach::None
                    }
                }
                cmd::FPRD | cmd::FPWR | cmd::FPRW | cmd::FRMW => {
                    if adp == device.station() {
                        Reach::Here { ado }
                    } else {
                        Reach::None
                    }
                }
                // Broadcasts count devices in the position field too.
                cmd::BRD | cmd::BWR | cmd::BRW => {
                    frame[at + 2..at + 4].copy_from_slice(&adp.wrapping_add(1).to_le_bytes());
                    Reach::All { ado }
                }
                cmd::LRD | cmd::LWR | cmd::LRW => Reach::Logical { address: logical },
                _ => Reach::None,
            };
            let mut acc = access(command);
            // Read-multiple-write: the addressed device reads, every other writes.
            if command == cmd::ARMW || command == cmd::FRMW {
                acc = match reach {
                    Reach::Here { .. } => Access { read: true, write: false },
                    _ => Access { read: false, write: true },
                };
            }
            let reach = match (command, reach) {
                (cmd::ARMW | cmd::FRMW, Reach::None) => Reach::All { ado },
                (_, r) => r,
            };
            if reach == Reach::None {
                continue;
            }
            let data = &mut frame[at + DATAGRAM_HEADER..at + DATAGRAM_HEADER + dlen];
            let add = device.datagram(reach, acc, data);
            let w = at + DATAGRAM_HEADER + dlen;
            let wkc = u16::from_le_bytes([frame[w], frame[w + 1]]).wrapping_add(add);
            frame[w..w + 2].copy_from_slice(&wkc.to_le_bytes());
        }
        device.frame_done();
    }
    // The first device sends the frame back out of its port 0 with the
    // source address's second bit set (ESC datasheet, Section I, "Frame
    // processing"): how a master tells its frame from the one that came back.
    if !devices.is_empty() {
        frame[6] |= 0x02;
    }
    Ok(())
}
