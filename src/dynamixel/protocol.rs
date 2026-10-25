//! Dynamixel Protocol 2.0, as servos speak it on a half-duplex serial bus:
//! the packets, their CRC and byte stuffing, the instructions a bus master
//! sends and the status packets the servos answer with. Bytes in, values
//! out: no I/O, no servo behaviour, no timing.
//!
//! From ROBOTIS' Protocol 2.0 page (the spec,
//! <https://emanual.robotis.com/docs/en/dxl/protocol2/>) and its CRC page
//! (<https://emanual.robotis.com/docs/en/dxl/crc/>):
//!
//! - **A packet** is the header `FF FF FD`, a reserved `00`, the id, a length,
//!   an instruction, its parameters and a CRC. The length is a little-endian
//!   `u16` counting the instruction, the parameters and the CRC.
//! - **The CRC** is CRC-16 with polynomial `0x8005`, starting from 0, over
//!   every byte before it. It goes out little-endian.
//! - **Byte stuffing** keeps a header from appearing inside a packet: after
//!   any `FF FF FD` in the instruction and parameters, the sender adds an
//!   `FD`. The length and the CRC count the added bytes. A receiver checks
//!   the CRC, then removes them.
//! - **Id `0xFE` is broadcast**: every device executes the packet. Devices
//!   have ids 0 to 252.
//! - **A status packet** is a device's answer: instruction `0x55`, an error
//!   byte, then parameters.
//! - **Fast Sync Read and Fast Bulk Read** are answered with one status
//!   packet from every device together, not stuffed (see [`FastReply`]).
//!
//! Two bus masters are second sources. rustypot, which the Microduck's
//! runtime is built on, at v1.8.0 (`src/dynamixel_protocol/v2.rs`,
//! <https://github.com/pollen-robotics/rustypot/blob/686ec6c830d87b7277ea0b3728bcbb8be4d233aa/src/dynamixel_protocol/v2.rs>),
//! for the fast replies. DynamixelSDK, ROBOTIS' own
//! (`protocol2_packet_handler.cpp`,
//! <https://github.com/ROBOTIS-GIT/DynamixelSDK/blob/a8d35d51c87e7d253ece1baa963f498a8d933fa0/c%2B%2B/src/dynamixel_sdk/protocol2_packet_handler.cpp>),
//! for stuffing's edge cases and the longest packet.

/// Packet id that every device executes.
pub const BROADCAST_ID: u8 = 0xFE;

/// The highest id a device can have (the spec, Packet ID).
pub const MAX_ID: u8 = 0xFC;

/// Every packet's first four bytes: the header, then the reserved byte.
pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];

/// The longest packet, header to CRC, in bytes. ROBOTIS doesn't say how long
/// a packet a servo takes. DynamixelSDK sends and takes 1024 at most
/// (`TXPACKET_MAX_LEN`, `RXPACKET_MAX_LEN`), so a longer one is taken for
/// noise.
pub const MAX_PACKET: usize = 1024;

/// The instructions, by number (the spec, Instruction).
pub mod instruction {
    pub const PING: u8 = 0x01;
    pub const READ: u8 = 0x02;
    pub const WRITE: u8 = 0x03;
    pub const REG_WRITE: u8 = 0x04;
    pub const ACTION: u8 = 0x05;
    pub const FACTORY_RESET: u8 = 0x06;
    pub const REBOOT: u8 = 0x08;
    pub const CLEAR: u8 = 0x10;
    /// A status packet: a device's answer.
    pub const STATUS: u8 = 0x55;
    pub const SYNC_READ: u8 = 0x82;
    pub const SYNC_WRITE: u8 = 0x83;
    pub const FAST_SYNC_READ: u8 = 0x8A;
    pub const BULK_READ: u8 = 0x92;
    pub const BULK_WRITE: u8 = 0x93;
    pub const FAST_BULK_READ: u8 = 0x9A;
}

/// A status packet's error byte (the spec, Error): the alert bit, and an
/// error number in the seven bits below it. The numbers are not bits: 3 is a
/// CRC error, not a result fail and an instruction error.
pub mod error {
    /// The device has a hardware error; its Hardware Error Status register
    /// says which.
    pub const ALERT: u8 = 0x80;
    /// It failed to process the instruction.
    pub const RESULT_FAIL: u8 = 0x01;
    /// An undefined instruction, or Action without Reg Write.
    pub const INSTRUCTION: u8 = 0x02;
    /// The packet's CRC didn't match.
    pub const CRC: u8 = 0x03;
    /// A value outside the register's range.
    pub const DATA_RANGE: u8 = 0x04;
    /// Fewer bytes than the register holds.
    pub const DATA_LENGTH: u8 = 0x05;
    /// A value outside the limit configured for the register.
    pub const DATA_LIMIT: u8 = 0x06;
    /// A read-only, write-only or undefined address, or an EEPROM register
    /// written with torque on.
    pub const ACCESS: u8 = 0x07;
}

/// The spec's CRC table: CRC-16, polynomial `0x8005`, most significant bit
/// first.
const CRC_TABLE: [u16; 256] = {
    let mut table = [0; 256];
    let mut i = 0;
    while i < 256 {
        let mut c = (i as u16) << 8;
        let mut bit = 0;
        while bit < 8 {
            c = if c & 0x8000 != 0 { (c << 1) ^ 0x8005 } else { c << 1 };
            bit += 1;
        }
        table[i] = c;
        i += 1;
    }
    table
};

/// The CRC of `bytes`, as the spec's `update_crc` computes it from 0.
pub fn crc(bytes: &[u8]) -> u16 {
    bytes.iter().fold(0, |c, &b| (c << 8) ^ CRC_TABLE[((c >> 8) as u8 ^ b) as usize])
}

/// Adds an `FD` after every `FF FF FD` in `body` (the spec, Packet Process).
/// The scan runs over `body` only, so an added `FD` never starts a new match:
/// `FF FF FD FD` becomes `FF FF FD FD FD`, as DynamixelSDK's `addStuffing`.
pub fn stuff(body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(body.len());
    for (i, &b) in body.iter().enumerate() {
        out.push(b);
        if i >= 2 && body[i - 2..=i] == [0xFF, 0xFF, 0xFD] {
            out.push(0xFD);
        }
    }
    out
}

/// Removes the `FD` after every `FF FF FD`: the inverse of [`stuff`], as
/// DynamixelSDK's `removeStuffing`.
pub fn unstuff(body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(body.len());
    let mut i = 0;
    while i < body.len() {
        out.push(body[i]);
        if i >= 2 && body[i - 2..=i] == [0xFF, 0xFF, 0xFD] && body.get(i + 1) == Some(&0xFD) {
            i += 1;
        }
        i += 1;
    }
    out
}

/// One packet, its parameters unstuffed. For a status packet, the error byte
/// is the first parameter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Packet {
    pub id: u8,
    pub instruction: u8,
    pub params: Vec<u8>,
}

impl Packet {
    /// The packet on the wire: stuffed, with its length and CRC.
    pub fn encode(&self) -> Vec<u8> {
        let mut body = vec![self.instruction];
        body.extend(&self.params);
        let body = stuff(&body);
        let mut out = HEADER.to_vec();
        out.push(self.id);
        out.extend((body.len() as u16 + 2).to_le_bytes());
        out.extend(body);
        out.extend(crc(&out).to_le_bytes());
        out
    }
}

/// Why the [`Decoder`] dropped what began as a packet. It then looks for the
/// next header from the byte after this one's start, so a packet inside it
/// is still found.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dropped {
    /// Not a device's id, nor broadcast: 253 or 255. The spec reads a
    /// packet only "if the Packet ID is valid".
    Id { id: u8 },
    /// A length below 3, too short for an instruction and a CRC.
    TooShort { id: u8, length: u16 },
    /// A packet longer than [`MAX_PACKET`].
    TooLong { id: u8, length: u16 },
    /// The CRC doesn't match the bytes: one was lost or changed.
    Crc { id: u8, expected: u16, got: u16 },
}

/// Finds packets in a serial byte stream, as a servo does (the spec,
/// Processing Order of Reception): look for a header, skipping `FF FF FD FD`,
/// which is stuffing; read the id and length; check the CRC; unstuff.
/// Bytes may come in any chunks, with noise between packets.
#[derive(Debug, Default)]
pub struct Decoder {
    buf: Vec<u8>,
}

impl Decoder {
    pub fn push(&mut self, bytes: &[u8]) {
        self.buf.extend_from_slice(bytes);
    }

    /// The next packet, or why one was dropped; `None` until more bytes come.
    pub fn next_packet(&mut self) -> Option<Result<Packet, Dropped>> {
        match self.buf.windows(HEADER.len()).position(|w| w == HEADER) {
            Some(start) => {
                self.buf.drain(..start);
            }
            None => {
                // The last three bytes may begin a header.
                let noise = self.buf.len().saturating_sub(HEADER.len() - 1);
                self.buf.drain(..noise);
                return None;
            }
        }
        let &[_, _, _, _, id, lo, hi, ..] = self.buf.as_slice() else { return None };
        let length = u16::from_le_bytes([lo, hi]);
        let end = 7 + length as usize;
        let dropped = if id > MAX_ID && id != BROADCAST_ID {
            Some(Dropped::Id { id })
        } else if length < 3 {
            Some(Dropped::TooShort { id, length })
        } else if end > MAX_PACKET {
            Some(Dropped::TooLong { id, length })
        } else if self.buf.len() < end {
            return None;
        } else {
            let (expected, got) =
                (crc(&self.buf[..end - 2]), u16::from_le_bytes([self.buf[end - 2], self.buf[end - 1]]));
            (expected != got).then_some(Dropped::Crc { id, expected, got })
        };
        if let Some(dropped) = dropped {
            self.buf.drain(..1);
            return Some(Err(dropped));
        }
        let body = unstuff(&self.buf[7..end - 2]);
        self.buf.drain(..end);
        Some(Ok(Packet { id, instruction: body[0], params: body[1..].to_vec() }))
    }
}

/// Where a Bulk Read or Fast Bulk Read reads, for one device: `length` bytes
/// from `address`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    pub id: u8,
    pub address: u16,
    pub length: u16,
}

/// What a Bulk Write writes, for one device: `data` at `address`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Chunk {
    pub id: u8,
    pub address: u16,
    pub data: Vec<u8>,
}

/// An instruction, with its parameters as the spec lays them out under
/// Instruction Details. Addresses and lengths are little-endian `u16`s.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Instruction {
    Ping,
    Read {
        address: u16,
        length: u16,
    },
    Write {
        address: u16,
        data: Vec<u8>,
    },
    /// A write kept until Action.
    RegWrite {
        address: u16,
        data: Vec<u8>,
    },
    Action,
    /// `option` is `0xFF` to reset everything, `0x01` all but the id, `0x02`
    /// all but the id and baud rate.
    FactoryReset {
        option: u8,
    },
    Reboot,
    /// `option` `0x01` with `key` `44 58 4C 22` resets the multi-turn
    /// position to within one turn.
    Clear {
        option: u8,
        key: [u8; 4],
    },
    /// Each id answers with its own status packet, in this order.
    SyncRead {
        address: u16,
        length: u16,
        ids: Vec<u8>,
    },
    /// Every write's data is `length` bytes.
    SyncWrite {
        address: u16,
        length: u16,
        writes: Vec<(u8, Vec<u8>)>,
    },
    /// Laid out as Sync Read; answered with one [`FastReply`].
    FastSyncRead {
        address: u16,
        length: u16,
        ids: Vec<u8>,
    },
    BulkRead(Vec<Region>),
    BulkWrite(Vec<Chunk>),
    /// Laid out as Bulk Read; answered with one [`FastReply`].
    FastBulkRead(Vec<Region>),
}

/// Why a packet's instruction can't be read.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Invalid {
    /// No instruction has this number, or not one this module reads.
    Unknown { instruction: u8 },
    /// The parameters aren't laid out as the spec says for this
    /// instruction. What a servo answers then, the spec doesn't say.
    Params { instruction: u8 },
}

impl Instruction {
    fn code(&self) -> u8 {
        match self {
            Instruction::Ping => instruction::PING,
            Instruction::Read { .. } => instruction::READ,
            Instruction::Write { .. } => instruction::WRITE,
            Instruction::RegWrite { .. } => instruction::REG_WRITE,
            Instruction::Action => instruction::ACTION,
            Instruction::FactoryReset { .. } => instruction::FACTORY_RESET,
            Instruction::Reboot => instruction::REBOOT,
            Instruction::Clear { .. } => instruction::CLEAR,
            Instruction::SyncRead { .. } => instruction::SYNC_READ,
            Instruction::SyncWrite { .. } => instruction::SYNC_WRITE,
            Instruction::FastSyncRead { .. } => instruction::FAST_SYNC_READ,
            Instruction::BulkRead(_) => instruction::BULK_READ,
            Instruction::BulkWrite(_) => instruction::BULK_WRITE,
            Instruction::FastBulkRead(_) => instruction::FAST_BULK_READ,
        }
    }

    fn params(&self) -> Vec<u8> {
        let mut p = Vec::new();
        match self {
            Instruction::Ping | Instruction::Action | Instruction::Reboot => {}
            Instruction::Read { address, length } => {
                p.extend(address.to_le_bytes());
                p.extend(length.to_le_bytes());
            }
            Instruction::Write { address, data } | Instruction::RegWrite { address, data } => {
                p.extend(address.to_le_bytes());
                p.extend(data);
            }
            Instruction::FactoryReset { option } => p.push(*option),
            Instruction::Clear { option, key } => {
                p.push(*option);
                p.extend(key);
            }
            Instruction::SyncRead { address, length, ids } | Instruction::FastSyncRead { address, length, ids } => {
                p.extend(address.to_le_bytes());
                p.extend(length.to_le_bytes());
                p.extend(ids);
            }
            Instruction::SyncWrite { address, length, writes } => {
                p.extend(address.to_le_bytes());
                p.extend(length.to_le_bytes());
                for (id, data) in writes {
                    p.push(*id);
                    p.extend(data);
                }
            }
            Instruction::BulkRead(regions) | Instruction::FastBulkRead(regions) => {
                for r in regions {
                    p.push(r.id);
                    p.extend(r.address.to_le_bytes());
                    p.extend(r.length.to_le_bytes());
                }
            }
            Instruction::BulkWrite(chunks) => {
                for c in chunks {
                    p.push(c.id);
                    p.extend(c.address.to_le_bytes());
                    p.extend((c.data.len() as u16).to_le_bytes());
                    p.extend(&c.data);
                }
            }
        }
        p
    }

    /// The instruction packet to device `id`, on the wire.
    pub fn encode(&self, id: u8) -> Vec<u8> {
        Packet { id, instruction: self.code(), params: self.params() }.encode()
    }

    pub fn decode(packet: &Packet) -> Result<Self, Invalid> {
        use instruction::*;
        let (code, p) = (packet.instruction, packet.params.as_slice());
        let u16_at = |i: usize| u16::from_le_bytes([p[i], p[i + 1]]);
        let invalid = Invalid::Params { instruction: code };
        Ok(match code {
            PING if p.is_empty() => Instruction::Ping,
            READ if p.len() == 4 => Instruction::Read { address: u16_at(0), length: u16_at(2) },
            WRITE if p.len() >= 2 => Instruction::Write { address: u16_at(0), data: p[2..].to_vec() },
            REG_WRITE if p.len() >= 2 => Instruction::RegWrite { address: u16_at(0), data: p[2..].to_vec() },
            ACTION if p.is_empty() => Instruction::Action,
            FACTORY_RESET if p.len() == 1 => Instruction::FactoryReset { option: p[0] },
            REBOOT if p.is_empty() => Instruction::Reboot,
            CLEAR if p.len() == 5 => Instruction::Clear { option: p[0], key: [p[1], p[2], p[3], p[4]] },
            SYNC_READ if p.len() >= 4 => {
                Instruction::SyncRead { address: u16_at(0), length: u16_at(2), ids: p[4..].to_vec() }
            }
            FAST_SYNC_READ if p.len() >= 4 => {
                Instruction::FastSyncRead { address: u16_at(0), length: u16_at(2), ids: p[4..].to_vec() }
            }
            SYNC_WRITE if p.len() >= 4 => {
                let length = u16_at(2);
                let stride = 1 + length as usize;
                if (p.len() - 4) % stride != 0 {
                    return Err(invalid);
                }
                let writes = p[4..].chunks(stride).map(|w| (w[0], w[1..].to_vec())).collect();
                Instruction::SyncWrite { address: u16_at(0), length, writes }
            }
            BULK_READ | FAST_BULK_READ => {
                let (regions, rest) = p.as_chunks::<5>();
                if !rest.is_empty() {
                    return Err(invalid);
                }
                let regions = regions
                    .iter()
                    .map(|&[id, a, b, c, d]| Region {
                        id,
                        address: u16::from_le_bytes([a, b]),
                        length: u16::from_le_bytes([c, d]),
                    })
                    .collect();
                if code == BULK_READ {
                    Instruction::BulkRead(regions)
                } else {
                    Instruction::FastBulkRead(regions)
                }
            }
            BULK_WRITE => {
                let mut chunks = Vec::new();
                let mut rest = p;
                while let [id, a, b, c, d, tail @ ..] = rest {
                    let length = u16::from_le_bytes([*c, *d]) as usize;
                    let data = tail.get(..length).ok_or(invalid)?;
                    chunks.push(Chunk { id: *id, address: u16::from_le_bytes([*a, *b]), data: data.to_vec() });
                    rest = &tail[length..];
                }
                if !rest.is_empty() {
                    return Err(invalid);
                }
                Instruction::BulkWrite(chunks)
            }
            PING | READ | WRITE | REG_WRITE | ACTION | FACTORY_RESET | REBOOT | CLEAR | SYNC_READ | FAST_SYNC_READ
            | SYNC_WRITE => return Err(invalid),
            _ => return Err(Invalid::Unknown { instruction: code }),
        })
    }
}

/// A device's answer: its id, an [`error`] byte, and the parameters, such as
/// the bytes read.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Status {
    pub id: u8,
    pub error: u8,
    pub params: Vec<u8>,
}

impl Status {
    pub fn encode(&self) -> Vec<u8> {
        let mut params = vec![self.error];
        params.extend(&self.params);
        Packet { id: self.id, instruction: instruction::STATUS, params }.encode()
    }

    pub fn decode(packet: &Packet) -> Option<Self> {
        let [error, params @ ..] = packet.params.as_slice() else { return None };
        (packet.instruction == instruction::STATUS).then(|| Status {
            id: packet.id,
            error: *error,
            params: params.to_vec(),
        })
    }
}

/// One device's part of a [`FastReply`].
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Answer {
    pub id: u8,
    pub error: u8,
    pub data: Vec<u8>,
}

/// The one status packet that answers a Fast Sync Read or a Fast Bulk Read,
/// from every device asked, in the order asked (the spec's Fast Sync Read
/// and Fast Bulk Read sections, and their examples):
///
/// ```text
/// FF FF FD 00 FE LEN_L LEN_H 55
///   ERR_1 ID_1 DATA_1 CRC_1
///   ...
///   ERR_n ID_n DATA_n CRC_n
/// ```
///
/// - The id is broadcast's.
/// - The first device's error sits where a status packet's error does; the
///   last device's CRC is the packet's.
/// - Each device's CRC covers the whole packet so far, from the header. The
///   spec's examples show it; rustypot (`parse_fast_sync_read_status`)
///   checks every one.
/// - The length is 1 for the instruction, plus 4 and the data's length for
///   each device.
/// - Nothing is stuffed: "Fast Sync Read Status Packet does not perform Byte
///   Stuffing". rustypot doesn't unstuff it either.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FastReply {
    /// At least one.
    pub answers: Vec<Answer>,
}

impl FastReply {
    pub fn encode(&self) -> Vec<u8> {
        assert!(!self.answers.is_empty(), "a fast reply needs a device to answer");
        let length: usize = 1 + self.answers.iter().map(|a| 4 + a.data.len()).sum::<usize>();
        let mut out = HEADER.to_vec();
        out.push(BROADCAST_ID);
        out.extend((length as u16).to_le_bytes());
        out.push(instruction::STATUS);
        for a in &self.answers {
            out.push(a.error);
            out.push(a.id);
            out.extend(&a.data);
            out.extend(crc(&out).to_le_bytes());
        }
        out
    }

    /// The answers in `bytes`, one packet, when each device sent the number
    /// of bytes in `lengths`; `None` if they don't fit or a CRC fails.
    pub fn decode(bytes: &[u8], lengths: &[u16]) -> Option<Self> {
        let size = 8 + lengths.iter().map(|&l| 4 + l as usize).sum::<usize>();
        if lengths.is_empty() || bytes.len() != size || bytes[..4] != HEADER {
            return None;
        }
        if bytes[4] != BROADCAST_ID || u16::from_le_bytes([bytes[5], bytes[6]]) as usize != size - 7 {
            return None;
        }
        if bytes[7] != instruction::STATUS {
            return None;
        }
        let mut at = 8;
        let mut answers = Vec::with_capacity(lengths.len());
        for &length in lengths {
            let crc_at = at + 2 + length as usize;
            if u16::from_le_bytes([bytes[crc_at], bytes[crc_at + 1]]) != crc(&bytes[..crc_at]) {
                return None;
            }
            answers.push(Answer { id: bytes[at + 1], error: bytes[at], data: bytes[at + 2..crc_at].to_vec() });
            at = crc_at + 2;
        }
        Some(FastReply { answers })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every packet the decoder finds in `bytes`, and every drop.
    fn decode_all(bytes: &[u8]) -> Vec<Result<Packet, Dropped>> {
        let mut d = Decoder::default();
        d.push(bytes);
        std::iter::from_fn(|| d.next_packet()).collect()
    }

    /// The one packet in `bytes`.
    fn decode_one(bytes: &[u8]) -> Packet {
        let mut found = decode_all(bytes);
        assert_eq!(found.len(), 1, "{found:?}");
        found.remove(0).unwrap()
    }

    /// Encodes as the spec's example, and decodes back.
    fn check(name: &str, i: Instruction, id: u8, example: &[u8]) {
        assert_eq!(i.encode(id), example, "{name}");
        let p = decode_one(example);
        assert_eq!(p.id, id, "{name}");
        assert_eq!(Instruction::decode(&p), Ok(i), "{name}");
    }

    #[test]
    fn the_crc_table_is_the_specs() {
        assert_eq!(CRC_TABLE[..4], [0x0000, 0x8005, 0x800F, 0x000A]);
        assert_eq!(CRC_TABLE[128], 0x8303);
        assert_eq!(CRC_TABLE[255], 0x0202);
    }

    // The examples below are the spec's, under Instruction Details.

    #[test]
    fn ping_read_and_write() {
        let ping = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4E];
        check("ping", Instruction::Ping, 1, &ping);
        let broadcast = [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x03, 0x00, 0x01, 0x31, 0x42];
        check("broadcast ping", Instruction::Ping, BROADCAST_ID, &broadcast);
        // Present Position, 4 bytes at 132.
        let read = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x84, 0x00, 0x04, 0x00, 0x1D, 0x15];
        check("read", Instruction::Read { address: 132, length: 4 }, 1, &read);
        // 512 to Goal Position, at 116.
        let write = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03, 0x74, 0x00, 0x00, 0x02, 0x00, 0x00, 0xCA, 0x89];
        check("write", Instruction::Write { address: 116, data: 512u32.to_le_bytes().to_vec() }, 1, &write);
    }

    #[test]
    fn reg_write_action_factory_reset_reboot_and_clear() {
        // 200 to Goal Velocity, at 104.
        let reg_write =
            [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x04, 0x68, 0x00, 0xC8, 0x00, 0x00, 0x00, 0xAE, 0x8E];
        check("reg write", Instruction::RegWrite { address: 104, data: 200u32.to_le_bytes().to_vec() }, 1, &reg_write);
        let action = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x05, 0x02, 0xCE];
        check("action", Instruction::Action, 1, &action);
        let reset = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x06, 0x01, 0xA1, 0xE6];
        check("factory reset", Instruction::FactoryReset { option: 0x01 }, 1, &reset);
        let reboot = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x03, 0x00, 0x08, 0x2F, 0x4E];
        check("reboot", Instruction::Reboot, 1, &reboot);
        let clear = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x10, 0x01, 0x44, 0x58, 0x4C, 0x22, 0xB1, 0xDC];
        check("clear", Instruction::Clear { option: 0x01, key: [0x44, 0x58, 0x4C, 0x22] }, 1, &clear);
    }

    #[test]
    fn sync_read_and_sync_write() {
        let sync_read =
            [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x82, 0x84, 0x00, 0x04, 0x00, 0x01, 0x02, 0xCE, 0xFA];
        check(
            "sync read",
            Instruction::SyncRead { address: 132, length: 4, ids: vec![1, 2] },
            BROADCAST_ID,
            &sync_read,
        );
        let sync_write = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00, 0x01, 0x96, 0x00, 0x00, 0x00, 0x02,
            0xAA, 0x00, 0x00, 0x00, 0x82, 0x87,
        ];
        let writes = vec![(1, 150u32.to_le_bytes().to_vec()), (2, 170u32.to_le_bytes().to_vec())];
        check("sync write", Instruction::SyncWrite { address: 116, length: 4, writes }, BROADCAST_ID, &sync_write);
        let fast =
            [0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0A, 0x00, 0x8A, 0x84, 0x00, 0x04, 0x00, 0x03, 0x07, 0x04, 0x20, 0xF2];
        check("fast sync read", Instruction::FastSyncRead { address: 132, length: 4, ids: vec![3, 7, 4] }, 0xFE, &fast);
    }

    #[test]
    fn bulk_read_and_bulk_write() {
        // Present Voltage (2 bytes at 144) from 1, Present Temperature (1 at 146) from 2.
        let bulk_read = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x0D, 0x00, 0x92, 0x01, 0x90, 0x00, 0x02, 0x00, 0x02, 0x92, 0x00, 0x01, 0x00,
            0x1A, 0x05,
        ];
        let regions = vec![Region { id: 1, address: 144, length: 2 }, Region { id: 2, address: 146, length: 1 }];
        check("bulk read", Instruction::BulkRead(regions), BROADCAST_ID, &bulk_read);
        // 160 to Max Voltage Limit (2 bytes at 32) on 1, 80 to Temperature Limit (1 at 31) on 2.
        let bulk_write = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x10, 0x00, 0x93, 0x01, 0x20, 0x00, 0x02, 0x00, 0xA0, 0x00, 0x02, 0x1F, 0x00,
            0x01, 0x00, 0x50, 0xB7, 0x68,
        ];
        let chunks =
            vec![Chunk { id: 1, address: 32, data: vec![0xA0, 0x00] }, Chunk { id: 2, address: 31, data: vec![0x50] }];
        check("bulk write", Instruction::BulkWrite(chunks), BROADCAST_ID, &bulk_write);
        // Present Position (4 at 132) from 3, Present PWM (2 at 124) from 7, Present Temperature (1 at 146) from 4.
        let fast = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x12, 0x00, 0x9A, 0x03, 0x84, 0x00, 0x04, 0x00, 0x07, 0x7C, 0x00, 0x02, 0x00,
            0x04, 0x92, 0x00, 0x01, 0x00, 0xDA, 0x2D,
        ];
        let regions = vec![
            Region { id: 3, address: 132, length: 4 },
            Region { id: 7, address: 124, length: 2 },
            Region { id: 4, address: 146, length: 1 },
        ];
        check("fast bulk read", Instruction::FastBulkRead(regions), BROADCAST_ID, &fast);
    }

    #[test]
    fn status_packets() {
        let examples: [(Status, &[u8]); 5] = [
            // A ping's answer: model number 1030, firmware 38.
            (
                Status { id: 1, error: 0, params: vec![0x06, 0x04, 0x26] },
                &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D],
            ),
            (
                Status { id: 2, error: 0, params: vec![0x06, 0x04, 0x26] },
                &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x6F, 0x6D],
            ),
            // A read's: 166.
            (
                Status { id: 1, error: 0, params: 166u32.to_le_bytes().to_vec() },
                &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0],
            ),
            // A write's.
            (
                Status { id: 1, error: 0, params: vec![] },
                &[0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x04, 0x00, 0x55, 0x00, 0xA1, 0x0C],
            ),
            // A bulk read's: 36 °C.
            (
                Status { id: 2, error: 0, params: vec![0x24] },
                &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x05, 0x00, 0x55, 0x00, 0x24, 0x8B, 0xA9],
            ),
        ];
        for (status, example) in examples {
            assert_eq!(status.encode(), example);
            assert_eq!(Status::decode(&decode_one(example)), Some(status));
        }
        let alert = Status { id: 7, error: error::ALERT | error::DATA_RANGE, params: vec![] };
        assert_eq!(Status::decode(&decode_one(&alert.encode())), Some(alert));
        assert_eq!(Status::decode(&decode_one(&Instruction::Ping.encode(1))), None, "not a status packet");
    }

    /// The spec's Fast Sync Read example: Present Position from 3, 7 and 4.
    const FAST_SYNC_REPLY: [u8; 32] = [
        0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x19, 0x00, 0x55, //
        0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x84, 0x08, //
        0x00, 0x07, 0x1F, 0x08, 0x00, 0x00, 0x16, 0xCA, //
        0x00, 0x04, 0xFF, 0x03, 0x00, 0x00, 0xD1, 0x9E,
    ];

    fn fast_sync_answers() -> Vec<Answer> {
        vec![
            Answer { id: 3, error: 0, data: 166u32.to_le_bytes().to_vec() },
            Answer { id: 7, error: 0, data: 2079u32.to_le_bytes().to_vec() },
            Answer { id: 4, error: 0, data: 1023u32.to_le_bytes().to_vec() },
        ]
    }

    #[test]
    fn a_fast_sync_read_is_answered_as_the_spec_example() {
        let reply = FastReply { answers: fast_sync_answers() };
        assert_eq!(reply.encode(), FAST_SYNC_REPLY);
        assert_eq!(FastReply::decode(&FAST_SYNC_REPLY, &[4, 4, 4]), Some(reply));
        // Each device's CRC covers the packet from its header, so the last is the packet's.
        for end in [14, 22, 30] {
            assert_eq!(crc(&FAST_SYNC_REPLY[..end]).to_le_bytes(), FAST_SYNC_REPLY[end..end + 2]);
        }
        assert_eq!(decode_one(&FAST_SYNC_REPLY).id, BROADCAST_ID, "a whole packet, by its length and CRC");
    }

    #[test]
    fn a_fast_bulk_read_is_answered_as_the_spec_example() {
        let example = [
            0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x14, 0x00, 0x55, //
            0x00, 0x03, 0xA6, 0x00, 0x00, 0x00, 0x67, 0xA4, //
            0x00, 0x07, 0xA5, 0x01, 0x24, 0x74, //
            0x00, 0x04, 0x1F, 0xD9, 0xC1,
        ];
        let reply = FastReply {
            answers: vec![
                Answer { id: 3, error: 0, data: 166u32.to_le_bytes().to_vec() },
                Answer { id: 7, error: 0, data: 421u16.to_le_bytes().to_vec() },
                Answer { id: 4, error: 0, data: vec![31] },
            ],
        };
        assert_eq!(reply.encode(), example);
        assert_eq!(FastReply::decode(&example, &[4, 2, 1]), Some(reply));
    }

    #[test]
    fn a_fast_reply_carries_each_devices_error_and_is_not_stuffed() {
        let mut answers = fast_sync_answers();
        answers[1].error = error::ALERT;
        answers[2].data = vec![0xFF, 0xFF, 0xFD, 0x00];
        let reply = FastReply { answers };
        let bytes = reply.encode();
        assert_eq!(bytes.len(), FAST_SYNC_REPLY.len(), "no stuffing");
        assert_eq!(bytes[16], error::ALERT);
        assert_eq!(FastReply::decode(&bytes, &[4, 4, 4]), Some(reply));
    }

    #[test]
    fn a_broken_fast_reply_is_refused() {
        assert_eq!(FastReply::decode(&FAST_SYNC_REPLY, &[4, 4]), None, "a device too few");
        assert_eq!(FastReply::decode(&FAST_SYNC_REPLY[..24], &[4, 4, 4]), None, "a device missing");
        assert_eq!(FastReply::decode(&FAST_SYNC_REPLY, &[4, 2, 6]), None, "devices cut wrong");
        let mut flipped = FAST_SYNC_REPLY;
        flipped[10] ^= 1;
        assert_eq!(FastReply::decode(&flipped, &[4, 4, 4]), None, "a changed byte");
    }

    #[test]
    fn stuffing() {
        assert_eq!(stuff(&[1, 0xFF, 0xFD, 0xFF]), [1, 0xFF, 0xFD, 0xFF], "no header, nothing added");
        assert_eq!(stuff(&[0xFF, 0xFF, 0xFD, 7]), [0xFF, 0xFF, 0xFD, 0xFD, 7]);
        assert_eq!(stuff(&[0xFF, 0xFF, 0xFF, 0xFD]), [0xFF, 0xFF, 0xFF, 0xFD, 0xFD]);
        assert_eq!(stuff(&[0xFF, 0xFF, 0xFD, 0xFD]), [0xFF, 0xFF, 0xFD, 0xFD, 0xFD]);
        for body in [
            vec![0xFF, 0xFF, 0xFD, 0xFD],
            vec![0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD],
            vec![0x10, 0xFF, 0xFF, 0xFD, 0x00, 0xFF, 0xFF, 0xFF, 0xFD, 0xFD, 0x20],
        ] {
            assert_eq!(unstuff(&stuff(&body)), body);
        }
    }

    #[test]
    fn a_header_in_the_data_is_stuffed_and_counted() {
        let write = Instruction::Write { address: 116, data: vec![0xFF, 0xFF, 0xFD, 0x00] };
        let bytes = write.encode(1);
        assert_eq!(bytes[5..7], [0x0A, 0x00], "3 + 6 parameters + 1 stuffed");
        assert_eq!(bytes[8..15], [0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x00]);
        assert_eq!(Instruction::decode(&decode_one(&bytes)), Ok(write));
        let status = Status { id: 1, error: 0, params: vec![0xFF, 0xFF, 0xFD, 0xA6] };
        assert_eq!(Status::decode(&decode_one(&status.encode())), Some(status));
    }

    /// Noise a bus might carry: a stuffed header, one with a bad reserved
    /// byte, and the start of a header.
    const NOISE: [u8; 12] = [0x00, 0x13, 0xFF, 0xFF, 0xFD, 0xFD, 0xFF, 0xFF, 0xFD, 0x01, 0xFF, 0xFF];

    fn stream() -> (Vec<u8>, Vec<Packet>) {
        let instructions = [
            (1, Instruction::Read { address: 132, length: 4 }),
            (BROADCAST_ID, Instruction::SyncWrite { address: 116, length: 4, writes: vec![(1, vec![0xFF; 4])] }),
            (200, Instruction::Write { address: 64, data: vec![0xFF, 0xFF, 0xFD, 0xFD] }),
            (BROADCAST_ID, Instruction::FastSyncRead { address: 132, length: 4, ids: (1..=15).collect() }),
        ];
        let mut bytes = NOISE.to_vec();
        let mut packets = Vec::new();
        for (id, i) in instructions {
            bytes.extend(i.encode(id));
            bytes.extend(&NOISE[..3]);
            packets.push(Packet { id, instruction: i.code(), params: i.params() });
        }
        (bytes, packets)
    }

    #[test]
    fn the_decoder_takes_a_stream_byte_by_byte() {
        let (bytes, packets) = stream();
        let mut d = Decoder::default();
        let mut found = Vec::new();
        for b in bytes {
            d.push(&[b]);
            while let Some(p) = d.next_packet() {
                found.push(p.unwrap());
            }
        }
        assert_eq!(found, packets);
    }

    #[test]
    fn the_decoder_takes_a_stream_in_odd_chunks() {
        let (bytes, packets) = stream();
        for sizes in [[2, 3, 5, 7], [11, 1, 13, 4], [64, 1, 1, 1]] {
            let mut d = Decoder::default();
            let mut found = Vec::new();
            let mut rest = bytes.as_slice();
            for &n in sizes.iter().cycle() {
                if rest.is_empty() {
                    break;
                }
                let (chunk, tail) = rest.split_at(n.min(rest.len()));
                d.push(chunk);
                rest = tail;
                while let Some(p) = d.next_packet() {
                    found.push(p.unwrap());
                }
            }
            assert_eq!(found, packets, "{sizes:?}");
            assert_eq!(d.buf.len(), 3, "the trailing noise might start a header");
        }
    }

    #[test]
    fn a_bad_crc_is_reported_and_skipped() {
        let ping = Instruction::Ping.encode(1);
        let mut bad = Instruction::Read { address: 132, length: 4 }.encode(2);
        let last = bad.len() - 1;
        bad[last] ^= 0xFF;
        let found = decode_all(&[ping.clone(), bad, ping].concat());
        assert_eq!(found.len(), 3);
        assert!(found[0].is_ok() && found[2].is_ok());
        assert!(matches!(found[1], Err(Dropped::Crc { id: 2, .. })), "{:?}", found[1]);
    }

    #[test]
    fn a_wrong_length_doesnt_swallow_the_next_packet() {
        let ping = Instruction::Ping.encode(1);
        // Claims 20 bytes after its length, which the ping and noise fill.
        let mut bytes = vec![0xFF, 0xFF, 0xFD, 0x00, 0x02, 20, 0x00, instruction::READ];
        bytes.extend(&ping);
        bytes.extend([0; 12]);
        let found = decode_all(&bytes);
        assert!(matches!(found[0], Err(Dropped::Crc { id: 2, .. })), "{found:?}");
        assert_eq!(found[1], Ok(decode_one(&ping)));
        assert_eq!(found.len(), 2);
    }

    #[test]
    fn bad_ids_and_lengths_are_reported() {
        let ping = Instruction::Ping.encode(1);
        let header = |id: u8, length: u16| [&HEADER[..], &[id], &length.to_le_bytes()].concat();
        let bytes = [header(0xFD, 3), header(1, 2), header(1, 1018), ping.clone()].concat();
        let found = decode_all(&bytes);
        assert_eq!(
            found,
            [
                Err(Dropped::Id { id: 0xFD }),
                Err(Dropped::TooShort { id: 1, length: 2 }),
                Err(Dropped::TooLong { id: 1, length: 1018 }),
                Ok(decode_one(&ping)),
            ]
        );
    }

    #[test]
    fn instructions_that_cant_be_read() {
        let packet = |instruction: u8, params: &[u8]| Packet { id: 1, instruction, params: params.to_vec() };
        let params = |instruction| Err(Invalid::Params { instruction });
        assert_eq!(Instruction::decode(&packet(0x20, &[])), Err(Invalid::Unknown { instruction: 0x20 }));
        assert_eq!(
            Instruction::decode(&packet(instruction::STATUS, &[0])),
            Err(Invalid::Unknown { instruction: 0x55 })
        );
        assert_eq!(Instruction::decode(&packet(instruction::PING, &[0])), params(instruction::PING));
        assert_eq!(Instruction::decode(&packet(instruction::READ, &[0x84, 0, 4])), params(instruction::READ));
        assert_eq!(Instruction::decode(&packet(instruction::WRITE, &[0x84])), params(instruction::WRITE));
        assert_eq!(Instruction::decode(&packet(instruction::CLEAR, &[1, 0x44])), params(instruction::CLEAR));
        // Two bytes for each of two ids, but one byte short.
        let sync_write = [0x74, 0, 2, 0, 1, 0xAA, 0xBB, 2, 0xCC];
        assert_eq!(Instruction::decode(&packet(instruction::SYNC_WRITE, &sync_write)), params(instruction::SYNC_WRITE));
        assert_eq!(
            Instruction::decode(&packet(instruction::BULK_READ, &[1, 0x90, 0, 2])),
            params(instruction::BULK_READ)
        );
        // Claims two bytes of data, has one.
        let bulk_write = [1, 0x20, 0, 2, 0, 0xA0];
        assert_eq!(Instruction::decode(&packet(instruction::BULK_WRITE, &bulk_write)), params(instruction::BULK_WRITE));
        let empty = Instruction::SyncWrite { address: 116, length: 4, writes: vec![] };
        assert_eq!(Instruction::decode(&decode_one(&empty.encode(BROADCAST_ID))), Ok(empty));
    }
}
