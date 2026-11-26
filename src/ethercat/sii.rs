//! What a device's EEPROM holds: the Slave Information Interface (SII). The
//! master reads it first, through the chip's EEPROM registers, to learn who
//! the device is and how to talk to it: its identity, its mailbox, its sync
//! managers, its default PDOs.
//!
//! Source: ETG.2010 (SII specification) by way of the layouts ethercrab and
//! SOEM parse, and the ESC datasheet, Section II (the first 8 words, which
//! the chip itself loads at power-on).

/// A PDO's entries, as the drive maps them by default: (object index,
/// subindex, bits, CoE data type).
#[derive(Debug, Clone)]
pub struct Pdo {
    pub index: u16,
    pub entries: Vec<(u16, u8, u8, u16)>,
}

impl Pdo {
    pub fn bytes(&self) -> usize {
        self.bits() / 8
    }
    pub fn bits(&self) -> usize {
        self.entries.iter().map(|e| e.2 as usize).sum()
    }
}

/// Where the sync managers' buffers are in the chip's RAM, and how long.
#[derive(Debug, Clone, Copy)]
pub struct Areas {
    pub mailbox_out: (u16, u16),
    pub mailbox_in: (u16, u16),
    pub outputs: (u16, u16),
    pub inputs: (u16, u16),
}

pub struct Identity {
    pub vendor: u32,
    pub product: u32,
    pub revision: u32,
    pub serial: u32,
    pub name: String,
}

/// CoE details (General category): SDOs, complete access, PDO assignment
/// and configuration, which the drive's mailbox supports; SDO information
/// it doesn't.
const COE_DETAILS: u8 = 0x01 | 0x04 | 0x08 | 0x20;

/// The EEPROM image: 2 KB (16 kbit), as the chips' smallest common one.
pub fn image(id: &Identity, areas: Areas, rx: &Pdo, tx: &Pdo) -> Vec<u8> {
    let mut words = [0u16; 0x40];
    // The chip's own settings: PDI control 0x05 (SPI, to the drive's
    // microcontroller), and the defaults of the rest.
    words[0] = 0x0005;
    words[7] = crc8(&words[..7]) as u16;
    let pair = |v: u32| [v as u16, (v >> 16) as u16];
    words[0x08..0x0A].copy_from_slice(&pair(id.vendor));
    words[0x0A..0x0C].copy_from_slice(&pair(id.product));
    words[0x0C..0x0E].copy_from_slice(&pair(id.revision));
    words[0x0E..0x10].copy_from_slice(&pair(id.serial));
    words[0x18] = areas.mailbox_out.0;
    words[0x19] = areas.mailbox_out.1;
    words[0x1A] = areas.mailbox_in.0;
    words[0x1B] = areas.mailbox_in.1;
    // Mailbox protocols: CoE (bit 2).
    words[0x1C] = 0x0004;
    // EEPROM size, kbit - 1; SII version 1.
    words[0x3E] = 15;
    words[0x3F] = 1;
    let mut bytes: Vec<u8> = words.iter().flat_map(|w| w.to_le_bytes()).collect();

    // Strings, numbered from 1: the device's name, its group, its PDOs'.
    let strings = [id.name.as_str(), "Drives", "RxPDO", "TxPDO"];
    let mut s = vec![strings.len() as u8];
    for t in strings {
        s.push(t.len() as u8);
        s.extend_from_slice(t.as_bytes());
    }
    category(&mut bytes, 10, &s);

    let mut general = vec![0u8; 32];
    general[0] = 2; // group
    general[2] = 1; // order: the name
    general[3] = 1; // name
    general[5] = COE_DETAILS;
    general[14] = 2;
    // Ports 0 and 1 are MII ("Y" each, 0x1 per nibble).
    general[16..18].copy_from_slice(&0x0011u16.to_le_bytes());
    category(&mut bytes, 30, &general);

    // FMMUs: outputs, inputs, the mailbox's status.
    category(&mut bytes, 40, &[0x01, 0x02, 0x03, 0x00]);

    let sm = |(start, len): (u16, u16), control: u8, kind: u8| {
        let mut b = start.to_le_bytes().to_vec();
        b.extend_from_slice(&len.to_le_bytes());
        b.extend_from_slice(&[control, 0, 1, kind]);
        b
    };
    let mut sms = Vec::new();
    // Mailbox out (written by the master), mailbox in, outputs (with the
    // watchdog), inputs: the control bytes of every CoE drive's ESI.
    sms.extend(sm(areas.mailbox_out, 0x26, 1));
    sms.extend(sm(areas.mailbox_in, 0x22, 2));
    sms.extend(sm(areas.outputs, 0x64, 3));
    sms.extend(sm(areas.inputs, 0x20, 4));
    category(&mut bytes, 41, &sms);

    category(&mut bytes, 50, &pdo(tx, 3, 4));
    category(&mut bytes, 51, &pdo(rx, 2, 3));
    bytes.extend_from_slice(&0xFFFFu16.to_le_bytes());
    bytes.resize(2048, 0xFF);
    bytes
}

/// A PDO category's data: the PDO, on sync manager `sm`, named string
/// `name`, then its entries.
fn pdo(p: &Pdo, sm: u8, name: u8) -> Vec<u8> {
    let mut b = p.index.to_le_bytes().to_vec();
    b.extend_from_slice(&[p.entries.len() as u8, sm, 0, name, 0, 0]);
    for &(index, sub, bits, kind) in &p.entries {
        b.extend_from_slice(&index.to_le_bytes());
        b.extend_from_slice(&[sub, 0, kind as u8, bits, 0, 0]);
    }
    b
}

/// A category: its type, its length in words, its data padded to a word.
fn category(bytes: &mut Vec<u8>, kind: u16, data: &[u8]) {
    let mut data = data.to_vec();
    if data.len() % 2 == 1 {
        data.push(0);
    }
    bytes.extend_from_slice(&kind.to_le_bytes());
    bytes.extend_from_slice(&((data.len() / 2) as u16).to_le_bytes());
    bytes.extend_from_slice(&data);
}

/// The checksum of the first 7 words: CRC-8, polynomial x⁸ + x² + x + 1,
/// starting at 0xFF (ESC datasheet, Section II, "SII EEPROM").
fn crc8(words: &[u16]) -> u8 {
    let mut crc = 0xFFu8;
    for b in words.iter().flat_map(|w| w.to_le_bytes()) {
        crc ^= b;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x07 } else { crc << 1 };
        }
    }
    crc
}
