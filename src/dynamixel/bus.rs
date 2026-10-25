//! The Dynamixel servos on one serial bus: who an instruction is for, and
//! what answers it, when.
//!
//! From the Protocol 2.0 spec (ROBOTIS e-Manual):
//!
//! - An instruction goes to one id, or to all of them (`0xFE`, broadcast).
//! - A servo answers with a status packet, after its Return Delay Time, as
//!   its Status Return Level allows: a ping always; reads at level 1 and 2;
//!   everything else at 2. Nothing answers a broadcast but a ping, which
//!   every servo answers in turn.
//! - A Sync Read or a Bulk Read is answered by each servo listed, in its
//!   own packet, one after the other, in the order listed. A Fast Sync Read
//!   or a Fast Bulk Read is answered in one packet, by all of them.
//! - The error byte carries an error number (refused writes and reads) and
//!   the alert bit, while the servo has a hardware error.
//!
//! A servo set to another baud rate than the bus's neither hears nor
//! answers. So, as on a real bus, a servo whose answer would be broken
//! (one left out of a fast read) breaks the whole answer.

use std::collections::HashSet;

use crate::dynamixel::xl330::{Refusal, Xl330};
use crate::dynamixel::{error, Answer, FastReply, Instruction, Invalid, Packet, Status, BROADCAST_ID};
use crate::robot::SERVO_BAUDS;

/// Bytes one or more servos send in answer to an instruction: ready
/// `after` seconds from the instruction's end, from servo `from` (none
/// when several sent them together).
#[derive(Debug, Clone, PartialEq)]
pub struct Reply {
    pub bytes: Vec<u8>,
    pub after: f64,
    pub from: Option<usize>,
}

pub struct Servos {
    servos: Vec<Xl330>,
    /// The bus's baud rate, as the Baud Rate register codes it.
    baud: u8,
    /// A write kept by Reg Write, per servo, until Action.
    registered: Vec<Option<(u16, Vec<u8>)>>,
    /// What has been said once, not to say it again.
    said: HashSet<String>,
}

impl Servos {
    pub fn new(servos: Vec<Xl330>, baud: u8) -> Self {
        let registered = vec![None; servos.len()];
        Self { servos, baud, registered, said: HashSet::new() }
    }

    pub fn servos(&self) -> &[Xl330] {
        &self.servos
    }

    pub fn servo_mut(&mut self, i: usize) -> &mut Xl330 {
        &mut self.servos[i]
    }

    /// The servos that hear `id`, in id order: one, or every one for a
    /// broadcast.
    fn hearing(&self, id: u8) -> Vec<usize> {
        let mut found: Vec<usize> = (0..self.servos.len())
            .filter(|&i| self.servos[i].baud() == self.baud && (id == BROADCAST_ID || self.servos[i].id() == id))
            .collect();
        found.sort_by_key(|&i| self.servos[i].id());
        found
    }

    /// The servo with `id` that hears the bus, if any.
    fn find(&self, id: u8) -> Option<usize> {
        self.hearing(id).into_iter().find(|_| id != BROADCAST_ID)
    }

    /// Seconds for `n` bytes to cross the bus.
    fn seconds(&self, n: usize) -> f64 {
        n as f64 * 10.0 / SERVO_BAUDS[self.baud as usize]
    }

    /// An instruction packet reached the servos: their answers.
    /// `link(i)` says whether servo `i` hears it (its link may be down);
    /// it's asked once for each servo the packet is for.
    pub fn deliver(&mut self, packet: &Packet, link: &mut dyn FnMut(usize) -> bool) -> Vec<Reply> {
        let instruction = match Instruction::decode(packet) {
            Ok(instruction) => instruction,
            Err(invalid) => return self.invalid(packet, invalid, link),
        };
        let id = packet.id;
        match instruction {
            Instruction::Ping => {
                let mut replies = Vec::new();
                let mut after = 0.0;
                for i in self.hearing(id).into_iter().filter(|&i| link(i)) {
                    let mut info = self.servos[i].read(0, 7).expect("in the table");
                    info.drain(2..6);
                    let bytes = self.status(i, 0, info);
                    after += self.servos[i].return_delay();
                    replies.push(Reply { after, from: Some(i), bytes: bytes.clone() });
                    after += self.seconds(bytes.len());
                }
                replies
            }
            Instruction::Read { address, length } => self.one(id, link, 1, |s| s.read(address, length)),
            Instruction::Write { address, data } => self.one(id, link, 2, |s| s.write(address, &data).map(|()| vec![])),
            Instruction::RegWrite { address, data } => {
                let targets = self.hearing(id);
                let ok = |s: &mut Xl330| s.check(address, &data).map(|()| vec![]);
                let replies = self.one(id, link, 2, ok);
                for i in targets {
                    if self.servos[i].check(address, &data).is_ok() {
                        self.registered[i] = Some((address, data.clone()));
                    }
                }
                replies
            }
            Instruction::Action => {
                let targets = self.hearing(id);
                for &i in &targets {
                    if let Some((address, data)) = self.registered[i].take() {
                        let _ = self.servos[i].write(address, &data);
                    }
                }
                self.one(id, link, 2, |_| Ok(vec![]))
            }
            Instruction::Reboot => {
                let replies = self.one(id, link, 2, |_| Ok(vec![]));
                for i in self.hearing(id) {
                    self.servos[i].reboot();
                    self.registered[i] = None;
                }
                replies
            }
            Instruction::FactoryReset { option } => {
                let replies = self.one(id, link, 2, |_| Ok(vec![]));
                for i in self.hearing(id) {
                    self.servos[i].factory_reset(option);
                }
                replies
            }
            // Single-turn position control keeps no turn count to clear.
            Instruction::Clear { .. } => self.one(id, link, 2, |_| Ok(vec![])),
            Instruction::SyncWrite { address, writes, .. } => {
                for (target, data) in writes {
                    if let Some(i) = self.find(target).filter(|&i| link(i)) {
                        let _ = self.servos[i].write(address, &data);
                    }
                }
                vec![]
            }
            Instruction::BulkWrite(chunks) => {
                for c in chunks {
                    if let Some(i) = self.find(c.id).filter(|&i| link(i)) {
                        let _ = self.servos[i].write(c.address, &c.data);
                    }
                }
                vec![]
            }
            Instruction::SyncRead { address, length, ids } => {
                let reads = ids.into_iter().map(|id| (id, address, length)).collect();
                self.in_turn(reads, link)
            }
            Instruction::BulkRead(regions) => {
                let reads = regions.into_iter().map(|r| (r.id, r.address, r.length)).collect();
                self.in_turn(reads, link)
            }
            Instruction::FastSyncRead { address, length, ids } => {
                let reads = ids.into_iter().map(|id| (id, address, length)).collect();
                self.together(reads, link)
            }
            Instruction::FastBulkRead(regions) => {
                let reads = regions.into_iter().map(|r| (r.id, r.address, r.length)).collect();
                self.together(reads, link)
            }
        }
    }

    /// The error byte for servo `i`, with `number` (0 for none), and the
    /// alert bit if it has a hardware error.
    fn error_byte(&self, i: usize, number: u8) -> u8 {
        number | if self.servos[i].hardware_error() != 0 { error::ALERT } else { 0 }
    }

    fn status(&self, i: usize, number: u8, params: Vec<u8>) -> Vec<u8> {
        Status { id: self.servos[i].id(), error: self.error_byte(i, number), params }.encode()
    }

    /// An instruction for one servo, done by `act`, answered if it's not a
    /// broadcast and its status return level is at least `level`.
    fn one(
        &mut self,
        id: u8,
        link: &mut dyn FnMut(usize) -> bool,
        level: u8,
        mut act: impl FnMut(&mut Xl330) -> Result<Vec<u8>, Refusal>,
    ) -> Vec<Reply> {
        let mut replies = Vec::new();
        for i in self.hearing(id).into_iter().filter(|&i| link(i)) {
            let done = act(&mut self.servos[i]);
            if id == BROADCAST_ID || self.servos[i].status_return_level() < level {
                continue;
            }
            let (number, params) = match done {
                Ok(params) => (0, params),
                Err(r) => (refusal(r), vec![]),
            };
            let after = self.servos[i].return_delay();
            replies.push(Reply { bytes: self.status(i, number, params), after, from: Some(i) });
        }
        replies
    }

    /// Reads answered each in its own packet, one servo after the other, in
    /// the order asked. A servo missing, or not hearing, doesn't answer; the
    /// next ones still do, after their own delay.
    fn in_turn(&mut self, reads: Vec<(u8, u16, u16)>, link: &mut dyn FnMut(usize) -> bool) -> Vec<Reply> {
        let mut replies = Vec::new();
        let mut after = 0.0;
        for (id, address, length) in reads {
            let Some(i) = self.find(id).filter(|&i| link(i)) else { continue };
            if self.servos[i].status_return_level() < 1 {
                continue;
            }
            let bytes = match self.servos[i].read(address, length) {
                Ok(data) => self.status(i, 0, data),
                Err(r) => self.status(i, refusal(r), vec![]),
            };
            after += self.servos[i].return_delay();
            replies.push(Reply { after, from: Some(i), bytes: bytes.clone() });
            after += self.seconds(bytes.len());
        }
        replies
    }

    /// Reads answered in one packet, by every servo asked. If one of them
    /// can't take its part, the packet is broken: no answer.
    fn together(&mut self, reads: Vec<(u8, u16, u16)>, link: &mut dyn FnMut(usize) -> bool) -> Vec<Reply> {
        let mut answers = Vec::new();
        let mut after = None;
        let mut whole = true;
        for (id, address, length) in reads {
            let Some(i) = self.find(id) else {
                whole = false;
                continue;
            };
            if !link(i) || self.servos[i].status_return_level() < 1 {
                whole = false;
                continue;
            }
            after.get_or_insert(self.servos[i].return_delay());
            let (number, data) = match self.servos[i].read(address, length) {
                Ok(data) => (0, data),
                Err(r) => (refusal(r), vec![0; length as usize]),
            };
            answers.push(Answer { id, error: self.error_byte(i, number), data });
        }
        match after {
            Some(after) if whole => vec![Reply { bytes: FastReply { answers }.encode(), after, from: None }],
            _ => vec![],
        }
    }

    /// A packet whose instruction drydock can't read: an unknown one is
    /// answered with the instruction error, as a servo does; one whose
    /// parameters don't fit, with the data length error.
    fn invalid(&mut self, packet: &Packet, invalid: Invalid, link: &mut dyn FnMut(usize) -> bool) -> Vec<Reply> {
        let (number, what) = match invalid {
            Invalid::Unknown { instruction } => (error::INSTRUCTION, format!("instruction {instruction:#04x}")),
            Invalid::Params { instruction } => (error::DATA_LENGTH, format!("a malformed {instruction:#04x}")),
        };
        if self.said.insert(what.clone()) {
            eprintln!("drydock: servo {}: {what}: answered with error {number}", packet.id);
        }
        if packet.id == BROADCAST_ID {
            return vec![];
        }
        let hearing = self.hearing(packet.id).into_iter().filter(|&i| link(i));
        hearing
            .filter(|&i| self.servos[i].status_return_level() == 2)
            .map(|i| Reply {
                bytes: self.status(i, number, vec![]),
                after: self.servos[i].return_delay(),
                from: Some(i),
            })
            .collect()
    }
}

fn refusal(r: Refusal) -> u8 {
    match r {
        Refusal::Access => error::ACCESS,
        Refusal::DataRange => error::DATA_RANGE,
        Refusal::DataLength => error::DATA_LENGTH,
        Refusal::DataLimit => error::DATA_LIMIT,
    }
}
