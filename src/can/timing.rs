//! How long frames take on a CAN bus, and in what order they cross it.
//!
//! A frame's length is counted bit by bit, as ISO 11898-1 lays it out: its
//! fields, its CRC, and the stuff bits that keep the bus from going six bits
//! without an edge, at the rate each part goes at (a CAN FD frame switches to
//! its data rate after its BRS bit, and back at its CRC delimiter).
//!
//! The bus carries one frame at a time. When it frees, the frames waiting
//! compete, and the lowest id wins (arbitration), wherever it comes from: a
//! master's commands, at low ids, go before the motors' answers. Frames from
//! one sender with the same id keep their order.

use crate::can::Frame;

/// A bus's bit rates, in bits per second: the nominal one, and CAN FD's
/// data phase.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rates {
    pub nominal: f64,
    pub data: f64,
}

/// A frame's bits: at the nominal rate, and at the data rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Bits {
    pub nominal: u32,
    pub data: u32,
}

impl Bits {
    pub fn seconds(&self, rates: Rates) -> f64 {
        self.nominal as f64 / rates.nominal + self.data as f64 / rates.data
    }
}

/// After the CRC: its delimiter, the ACK slot and delimiter, the end of
/// frame (7) and the space between frames (3), all at the nominal rate.
const TAIL: u32 = 1 + 2 + 7 + 3;

/// The bits of a data frame with an 11-bit id, stuffing included. A CAN FD
/// frame is taken to switch its bit rate (BRS), as Damiao's do.
pub fn bits(frame: &Frame) -> Bits {
    if frame.fd {
        fd_bits(frame)
    } else {
        classic_bits(frame)
    }
}

fn push(stream: &mut Vec<bool>, value: u32, width: u32) {
    for i in (0..width).rev() {
        stream.push(value >> i & 1 == 1);
    }
}

/// The header and data of a frame, before stuffing: SOF, the id, the
/// control field and the data, most significant bit first.
fn header_and_data(frame: &Frame, control: &[(u32, u32)], data_len: usize) -> Vec<bool> {
    let mut s = vec![false]; // SOF
    push(&mut s, frame.id as u32, 11);
    for &(value, width) in control {
        push(&mut s, value, width);
    }
    for &byte in &frame.data[..data_len] {
        push(&mut s, byte as u32, 8);
    }
    s
}

/// Where stuff bits go in `bits`: one after every five equal bits, itself
/// starting the next run. Returns the stuff bits' positions, as "after this
/// many of the original bits".
fn stuff_positions(bits: &[bool]) -> Vec<usize> {
    let mut positions = Vec::new();
    let (mut last, mut run) = (None, 0);
    for (i, &b) in bits.iter().enumerate() {
        if Some(b) == last {
            run += 1;
        } else {
            (last, run) = (Some(b), 1);
        }
        if run == 5 {
            positions.push(i + 1);
            (last, run) = (Some(!b), 1);
        }
    }
    positions
}

fn crc15(bits: &[bool]) -> u32 {
    let mut crc = 0u32;
    for &b in bits {
        let next = b ^ (crc >> 14 & 1 == 1);
        crc = crc << 1 & 0x7FFF;
        if next {
            crc ^= 0x4599;
        }
    }
    crc
}

fn classic_bits(frame: &Frame) -> Bits {
    let len = frame.len.min(8) as usize;
    // RTR, IDE, r0, then the DLC.
    let mut s = header_and_data(frame, &[(0, 3), (len as u32, 4)], len);
    let crc = crc15(&s);
    push(&mut s, crc, 15);
    let stuffed = s.len() + stuff_positions(&s).len();
    Bits { nominal: stuffed as u32 + TAIL, data: 0 }
}

/// CAN FD's data lengths, and the DLC codes above 8.
fn fd_length(len: u8) -> (usize, u32) {
    match len {
        0..=8 => (len as usize, len as u32),
        9..=12 => (12, 9),
        13..=16 => (16, 10),
        17..=20 => (20, 11),
        21..=24 => (24, 12),
        25..=32 => (32, 13),
        33..=48 => (48, 14),
        _ => (64, 15),
    }
}

fn fd_bits(frame: &Frame) -> Bits {
    let (len, dlc) = fd_length(frame.len);
    // RRS, IDE, FDF, res, BRS | ESI, then the DLC.
    let s = header_and_data(frame, &[(0b00101, 5), (0, 1), (dlc, 4)], len);
    // The rate switches after BRS: SOF, 11 id bits, and those five.
    const BRS_END: usize = 1 + 11 + 5;
    let stuffs = stuff_positions(&s);
    let nominal_stuffs = stuffs.iter().filter(|&&p| p < BRS_END).count();
    let data_stuffs = stuffs.len() - nominal_stuffs;
    // The stuff count (4 bits) and the CRC, with a fixed stuff bit before
    // the first and after every fourth bit after it.
    let crc_len: usize = if len <= 16 { 17 } else { 21 };
    let crc_field = 4 + crc_len;
    let fixed = crc_field.div_ceil(4);
    Bits {
        nominal: (BRS_END + nominal_stuffs) as u32 + TAIL,
        data: (s.len() - BRS_END + data_stuffs + crc_field + fixed) as u32,
    }
}

/// Where a frame comes from, and so where it goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum From {
    /// The bus master: it goes to the motors.
    Master,
    /// A motor: it goes to the master.
    Motor,
}

#[derive(Debug, Clone, Copy)]
struct Pending {
    frame: Frame,
    from: From,
    /// When it was ready to go.
    ready: f64,
    /// Order among equals.
    seq: u64,
}

#[derive(Debug, Clone, Copy)]
struct OnTheWire {
    pending: Pending,
    end: f64,
}

/// One bus: the frames waiting for it, the one crossing it, and how busy
/// it's been. Times are seconds on any one clock.
#[derive(Debug)]
pub struct Timeline {
    rates: Rates,
    waiting: Vec<Pending>,
    wire: Option<OnTheWire>,
    /// When the bus is next free.
    free: f64,
    seq: u64,
    /// Since the last report: time the bus was busy, and the longest wait.
    since: f64,
    busy: f64,
    longest_wait: f64,
}

impl Timeline {
    pub fn new(rates: Rates) -> Self {
        Self { rates, waiting: Vec::new(), wire: None, free: 0.0, seq: 0, since: 0.0, busy: 0.0, longest_wait: 0.0 }
    }

    /// A frame ready to go at `ready`.
    pub fn push(&mut self, frame: Frame, from: From, ready: f64) {
        self.seq += 1;
        self.waiting.push(Pending { frame, from, ready, seq: self.seq });
    }

    /// When something next happens: a frame starts, or one arrives.
    pub fn next_event(&self) -> Option<f64> {
        match self.wire {
            Some(w) => Some(w.end),
            None => self.waiting.iter().map(|p| p.ready.max(self.free)).reduce(f64::min),
        }
    }

    /// Runs the bus up to `now`, and gives the frames that arrived, with when
    /// they did. A caller pushing answers to them calls again: they may be
    /// due already.
    pub fn advance(&mut self, now: f64) -> Vec<(Frame, From, f64)> {
        let mut arrived = Vec::new();
        loop {
            if let Some(w) = self.wire {
                if w.end > now {
                    break;
                }
                arrived.push((w.pending.frame, w.pending.from, w.end));
                self.wire = None;
                continue;
            }
            let Some(start) = self.waiting.iter().map(|p| p.ready.max(self.free)).reduce(f64::min) else { break };
            if start > now {
                break;
            }
            // Arbitration: of those ready when the bus frees, the lowest id.
            let i = (0..self.waiting.len())
                .filter(|&i| self.waiting[i].ready <= start)
                .min_by_key(|&i| (self.waiting[i].frame.id, self.waiting[i].seq))
                .unwrap();
            let pending = self.waiting.swap_remove(i);
            let length = bits(&pending.frame).seconds(self.rates);
            let end = start + length;
            self.busy += length;
            self.longest_wait = self.longest_wait.max(start - pending.ready);
            self.free = end;
            self.wire = Some(OnTheWire { pending, end });
        }
        arrived
    }

    /// Once a second at most: how busy the bus was, if it was busy enough
    /// to matter (frames waited a millisecond or more, or the bus was
    /// carrying frames 90% of the time).
    pub fn report(&mut self, now: f64) -> Option<String> {
        let span = now - self.since;
        if span < 1.0 {
            return None;
        }
        let (load, wait) = (self.busy / span, self.longest_wait);
        (self.since, self.busy, self.longest_wait) = (now, 0.0, 0.0);
        (load >= 0.9 || wait >= 1e-3).then(|| {
            format!(
                "busy {:.0}% of the last {span:.1} s; frames waited up to {:.2} ms for it",
                load * 100.0,
                wait * 1e3
            )
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATES: Rates = Rates { nominal: 1e6, data: 5e6 };

    fn frame(id: u16, data: [u8; 8]) -> Frame {
        Frame::new(id, data)
    }

    #[test]
    fn a_classic_frame_of_eight_bytes_is_111_to_135_bits() {
        // Alternating bits: nothing to stuff in the data.
        let quiet = bits(&frame(0x155, [0x55; 8]));
        // Long runs: a stuff bit every four after the first five.
        let loud = bits(&frame(0x000, [0x00; 8]));
        for b in [quiet, loud] {
            assert!((111..=135).contains(&b.nominal), "{b:?}");
            assert_eq!(b.data, 0);
        }
        assert!(loud.nominal > quiet.nominal + 15, "{loud:?} {quiet:?}");
    }

    #[test]
    fn stuffing_follows_runs_of_five() {
        assert_eq!(stuff_positions(&[false; 5]), [5]);
        // The stuff bit is the other value: zeros after it start afresh…
        assert_eq!(stuff_positions(&[false; 10]), [5, 10]);
        // …and ones after it continue its run: four more make five.
        let mut bits = vec![false; 5];
        bits.extend([true; 4]);
        assert_eq!(stuff_positions(&bits), [5, 9]);
        assert_eq!(stuff_positions(&[false, true, false, true, false]), Vec::<usize>::new());
    }

    #[test]
    fn the_crc_matches_a_known_frame() {
        // Id 0x123, DLC 1, data 0xAB.
        let f = Frame { len: 1, ..frame(0x123, [0xAB, 0, 0, 0, 0, 0, 0, 0]) };
        let s = header_and_data(&f, &[(0, 3), (1, 4)], 1);
        assert_eq!(crc15(&s), crc15_reference(&s));
    }

    /// The CRC as a polynomial division over the message followed by 15
    /// zeros: another way to the same number.
    fn crc15_reference(bits: &[bool]) -> u32 {
        let mut m: Vec<bool> = bits.to_vec();
        m.extend([false; 15]);
        let poly = 0xC599u32; // x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
        for i in 0..bits.len() {
            if m[i] {
                for j in 0..16 {
                    m[i + j] ^= poly >> (15 - j) & 1 == 1;
                }
            }
        }
        m[bits.len()..].iter().fold(0, |acc, &b| acc << 1 | b as u32)
    }

    #[test]
    fn a_can_fd_frame_of_eight_bytes_goes_over_twice_as_fast() {
        let classic = frame(0x01, [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0]);
        let fd = classic.fd(true);
        let (c, f) = (bits(&classic).seconds(RATES), bits(&fd).seconds(RATES));
        assert!((111e-6..=135e-6).contains(&c), "{c}");
        assert!((45e-6..=60e-6).contains(&f), "{f}");
        let b = bits(&fd);
        assert!((30..=34).contains(&b.nominal), "{b:?}");
        // ESI, DLC, 64 data bits, stuff count, CRC-17, six fixed stuff bits.
        assert!(b.data >= 1 + 4 + 64 + 4 + 17 + 6, "{b:?}");
    }

    #[test]
    fn commands_win_arbitration_over_answers() {
        // Two commands from the master, and an answer ready as the first
        // command lands: the second command still goes first.
        let mut bus = Timeline::new(RATES);
        bus.push(frame(0x01, [0; 8]), From::Master, 0.0);
        bus.push(frame(0x02, [0; 8]), From::Master, 0.0);
        // The first starts at once, and lands when it's across.
        assert!(bus.advance(0.0).is_empty());
        let landed = bus.advance(bus.next_event().unwrap());
        assert_eq!(landed[0].0.id, 0x01);
        bus.push(frame(0x11, [0; 8]), From::Motor, landed[0].2);
        let order: Vec<u16> = bus.advance(1.0).iter().map(|(f, _, _)| f.id).collect();
        assert_eq!(order, [0x02, 0x11]);
    }

    #[test]
    fn frames_go_one_at_a_time() {
        let mut bus = Timeline::new(RATES);
        for id in 1..=8 {
            bus.push(frame(id, [0; 8]), From::Master, 0.0);
        }
        let arrived = bus.advance(1.0);
        let ends: Vec<f64> = arrived.iter().map(|a| a.2).collect();
        assert!(ends.windows(2).all(|w| w[1] > w[0] + 100e-6), "{ends:?}");
        // Eight classic frames: about a millisecond.
        assert!((0.9e-3..1.1e-3).contains(&ends[7]), "{}", ends[7]);
    }

    #[test]
    fn nothing_arrives_before_its_time() {
        let mut bus = Timeline::new(RATES);
        bus.push(frame(0x01, [0; 8]), From::Master, 0.0);
        assert!(bus.advance(50e-6).is_empty());
        let end = bus.next_event().unwrap();
        assert!(end > 100e-6);
        assert_eq!(bus.advance(end).len(), 1);
        assert_eq!(bus.next_event(), None);
    }
}
