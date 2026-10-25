//! How long bytes take on a half-duplex serial line, and in what order they
//! cross it.
//!
//! A byte is ten bits on the line (a start bit, eight data bits, a stop
//! bit), at the line's baud rate. One sender at a time: what's sent while
//! the line is busy waits for it, first come first served. A real line
//! would garble two senders at once; drydock's masters and servos take
//! turns, as Dynamixel's protocol has them, so this doesn't arise.

use std::collections::VecDeque;

/// Who sent bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum From {
    Master,
    Device,
}

/// Bytes that crossed the line: who sent them, when they were ready, and
/// when they started and finished crossing, in seconds on the run's clock.
#[derive(Debug, Clone, PartialEq)]
pub struct Crossed {
    pub bytes: Vec<u8>,
    pub from: From,
    pub ready: f64,
    pub start: f64,
    pub end: f64,
}

/// Bits on the line per byte.
pub const BITS_PER_BYTE: f64 = 10.0;

/// One serial line.
pub struct Line {
    baud: f64,
    waiting: VecDeque<(Vec<u8>, From, f64)>,
    /// When the line is free.
    free: f64,
    /// For the busy report: since when, how long it was busy, the longest
    /// anything waited.
    since: f64,
    busy: f64,
    longest_wait: f64,
}

impl Line {
    pub fn new(baud: f64) -> Self {
        Self { baud, waiting: VecDeque::new(), free: 0.0, since: 0.0, busy: 0.0, longest_wait: 0.0 }
    }

    /// How long `n` bytes take to cross.
    pub fn seconds(&self, n: usize) -> f64 {
        n as f64 * BITS_PER_BYTE / self.baud
    }

    /// Bytes ready to go at `ready`.
    pub fn push(&mut self, bytes: Vec<u8>, from: From, ready: f64) {
        if !bytes.is_empty() {
            self.waiting.push_back((bytes, from, ready));
        }
    }

    /// When the next bytes will have crossed, if any wait.
    pub fn next_event(&self) -> Option<f64> {
        let (bytes, _, ready) = self.waiting.front()?;
        Some(ready.max(self.free) + self.seconds(bytes.len()))
    }

    /// The bytes that have crossed by `now`, in order.
    pub fn advance(&mut self, now: f64) -> Vec<Crossed> {
        let mut crossed = Vec::new();
        while let Some(end) = self.next_event().filter(|&end| end <= now) {
            let (bytes, from, ready) = self.waiting.pop_front().expect("next_event saw it");
            let start = ready.max(self.free);
            self.longest_wait = self.longest_wait.max(start - ready);
            self.busy += end - start;
            self.free = end;
            crossed.push(Crossed { bytes, from, ready, start, end });
        }
        crossed
    }

    /// Once a second at most, what the line was like, if it was close to
    /// full or kept bytes waiting a millisecond.
    pub fn report(&mut self, now: f64) -> Option<String> {
        let span = now - self.since;
        if span < 1.0 {
            return None;
        }
        let (load, wait) = (self.busy / span, self.longest_wait);
        (self.since, self.busy, self.longest_wait) = (now, 0.0, 0.0);
        (load >= 0.9 || wait >= 1e-3).then(|| {
            format!("busy {:.0}% of the last {span:.1} s; bytes waited up to {:.2} ms for it", load * 100.0, wait * 1e3)
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_byte_is_ten_bits_one_sender_at_a_time() {
        let mut line = Line::new(1e6);
        line.push(vec![0; 10], From::Master, 0.0);
        line.push(vec![0; 5], From::Device, 0.0);
        assert!(line.advance(99e-6).is_empty());
        let first = line.advance(100e-6);
        assert_eq!((first.len(), first[0].end), (1, 100e-6));
        let second = line.advance(1.0);
        assert_eq!(second[0].from, From::Device);
        assert!((second[0].start - 100e-6).abs() < 1e-12 && (second[0].end - 150e-6).abs() < 1e-12);
    }
}
