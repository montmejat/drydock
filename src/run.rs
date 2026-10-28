//! How time passes: with the wall clock, frames crossing each bus as long
//! as they would; or with the bus masters' cycles.

use std::fs::File;
use std::io::{self, BufWriter, Write};
use std::path::Path;
use std::time::{Duration, Instant};

use crate::can::timing::{Crossed, From, Rates, Timeline};
use std::os::fd::{AsRawFd, RawFd};

use crate::can::Socket;
use crate::dock::Drydock;
use crate::dynamixel::{Decoder, Dropped};
use crate::poll::wait_any;
use crate::scenario::{Change, Fault, Happening};
use crate::serial::{timing as serial_timing, Pty};

/// Behind the wall clock by more than this, the simulation stops trying to
/// catch up, and says so.
const MAX_LAG: Duration = Duration::from_millis(100);

/// A bus as the clock sees it: its socket, and its timing.
pub struct Line {
    pub name: String,
    pub socket: Socket,
    pub rates: Rates,
    /// From a frame reaching a motor to its answer being ready, seconds.
    pub reply: f64,
}

/// A serial bus as the clock sees it: its pty, its baud rate, and the
/// packets coming together from its bytes.
pub struct SerialLine {
    pub pty: Pty,
    pub baud: f64,
    pub decoder: Decoder,
    /// When the master's last bytes finished crossing.
    pub last_byte: f64,
}

/// A servo drops a packet whose bytes stop coming for this long (the
/// Protocol 2.0 spec's 1.5 ms between bytes).
const BYTE_GAP: f64 = 1.5e-3;

impl SerialLine {
    pub fn new(pty: Pty, baud: f64) -> Self {
        Self { pty, baud, decoder: Decoder::default(), last_byte: 0.0 }
    }

    pub fn name(&self) -> String {
        self.pty.link().display().to_string()
    }

    /// Bytes from the master, crossed by `end`, after the last ones ended
    /// at `start`'s gap: the packets they complete.
    fn packets(&mut self, bytes: &[u8], start: f64, end: f64, said: &mut Said) -> Vec<crate::dynamixel::Packet> {
        if start - self.last_byte > BYTE_GAP {
            self.decoder = Decoder::default();
        }
        self.last_byte = end;
        self.decoder.push(bytes);
        let mut packets = Vec::new();
        while let Some(next) = self.decoder.next_packet() {
            match next {
                Ok(packet) => packets.push(packet),
                Err(dropped) => said.once(format!("{}: a packet dropped: {}", self.name(), describe(dropped))),
            }
        }
        packets
    }
}

fn describe(d: Dropped) -> String {
    match d {
        Dropped::Crc { id, expected, got } => {
            format!("to id {id}, its CRC is {got:#06x} where its bytes make {expected:#06x}")
        }
        other => format!("{other:?}"),
    }
}

/// What has been said once, not to say it again.
#[derive(Default)]
struct Said(std::collections::HashSet<String>);

impl Said {
    fn once(&mut self, message: String) {
        if self.0.insert(message.clone()) {
            eprintln!("drydock: {message}");
        }
    }
}

/// Every frame that crosses a bus, as a line of JSON: which bus, its id,
/// who sent it, its data, and when it was ready, started across and arrived,
/// in nanoseconds on the machine's monotonic clock (`CLOCK_MONOTONIC`), the
/// clock keel's traces use, so the two line up.
pub struct BusTrace {
    out: BufWriter<File>,
    /// The monotonic clock, in ns, when the run's clock read 0.
    base_ns: f64,
    flushed: f64,
}

impl BusTrace {
    pub fn create(path: &Path) -> io::Result<Self> {
        Ok(Self { out: BufWriter::new(File::create(path)?), base_ns: 0.0, flushed: 0.0 })
    }

    fn write(&mut self, bus: &str, c: &Crossed) -> io::Result<()> {
        let from = match c.from {
            From::Master => "master",
            From::Motor => "motor",
        };
        let data: String = c.frame.data[..c.frame.len as usize].iter().map(|b| format!("{b:02x}")).collect();
        let ns = |t: f64| (self.base_ns + t * 1e9).round() as u64;
        writeln!(
            self.out,
            r#"{{"bus":"{bus}","id":{},"from":"{from}","fd":{},"data":"{data}","ready_ns":{},"start_ns":{},"end_ns":{}}}"#,
            c.frame.id,
            c.frame.fd,
            ns(c.ready),
            ns(c.start),
            ns(c.end)
        )
    }

    /// Bytes that crossed a serial bus, as frames are written: the bus (its
    /// port), who sent them, their bytes, and when.
    fn write_serial(&mut self, port: &str, c: &serial_timing::Crossed) -> io::Result<()> {
        let from = match c.from {
            serial_timing::From::Master => "master",
            serial_timing::From::Device => "device",
        };
        let data: String = c.bytes.iter().map(|b| format!("{b:02x}")).collect();
        let ns = |t: f64| (self.base_ns + t * 1e9).round() as u64;
        writeln!(
            self.out,
            r#"{{"serial":"{port}","from":"{from}","data":"{data}","ready_ns":{},"start_ns":{},"end_ns":{}}}"#,
            ns(c.ready),
            ns(c.start),
            ns(c.end)
        )
    }

    /// A fault starting or ending, `at` on the run's clock (seconds), to
    /// `joint`'s motor or to the supply.
    fn write_happening(&mut self, joint: &str, h: &Happening, at: f64) -> io::Result<()> {
        let ns = (self.base_ns + at * 1e9).round() as u64;
        let what = match h.what {
            Change::Breaks { until: Some(u) } => format!(r#""breaks","for_s":{}"#, u - h.at),
            Change::Breaks { until: None } => r#""breaks""#.into(),
            Change::Recovers => r#""recovers""#.into(),
        };
        let fault = match h.fault {
            Fault::Motor { breakdown, .. } => format!(r#""joint":"{joint}","fault":"{}""#, breakdown.name()),
            Fault::Supply { volts } => format!(r#""supply_v":{volts}"#),
        };
        writeln!(self.out, r#"{{{fault},"what":{what},"sim_s":{},"at_ns":{ns}}}"#, h.at)
    }

    /// Out to the file every tenth of a second: drydock ends when it's
    /// killed, and loses only that much.
    fn flush_now_and_then(&mut self, now: f64) -> io::Result<()> {
        if now - self.flushed >= 0.1 {
            self.flushed = now;
            self.out.flush()?;
        }
        Ok(())
    }
}

fn monotonic_ns() -> f64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: writes into a local.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as f64 * 1e9 + ts.tv_nsec as f64
}

/// What the scenario did since the last step, in the log, and in the trace
/// if there's one (`at`: the run's clock, seconds).
fn report(dock: &mut Drydock, trace: Option<&mut BusTrace>, at: f64) -> io::Result<()> {
    let happenings = dock.happenings();
    let mut trace = trace;
    for h in &happenings {
        let (joint, what) = match h.fault {
            Fault::Motor { motor, breakdown } => {
                let name = breakdown.name();
                let what = match h.what {
                    Change::Breaks { until: Some(u) } => format!("breaks: {name}, for {:.3} s", u - h.at),
                    Change::Breaks { until: None } => format!("breaks: {name}"),
                    Change::Recovers => format!("recovers from {name}"),
                };
                (dock.joint_name(motor).to_owned(), what)
            }
            Fault::Supply { volts } => {
                let (to, back) = match volts {
                    0.0 => ("off".to_string(), "back on".to_string()),
                    v => (format!("to {v} V"), format!("back from {v} V")),
                };
                let what = match h.what {
                    Change::Breaks { until: Some(u) } => format!("goes {to}, for {:.3} s", u - h.at),
                    Change::Breaks { until: None } => format!("goes {to}"),
                    Change::Recovers => format!("is {back}"),
                };
                ("the supply".to_owned(), what)
            }
        };
        eprintln!("drydock: {:.3} s: {joint} {what}", h.at);
        if let Some(t) = trace.as_deref_mut() {
            t.write_happening(&joint, h, at)?;
        }
    }
    Ok(())
}

/// In real time: a step of physics each timestep of the wall clock. A frame
/// from a master crosses its bus as long as it would on a real one, after
/// the frames ahead of it, and reaches the motors when it's across; their
/// answers cross it in turn, and reach the master when they're across. On a
/// serial bus, bytes do the same, ten bits each at its baud rate, and a
/// servo answers a packet once it's across, after its return delay.
///
/// With `trace`, every frame and every serial byte that crosses a bus goes
/// there too.
pub fn realtime(
    mut dock: Drydock,
    lines: &[Line],
    serial: &mut [SerialLine],
    mut trace: Option<BusTrace>,
) -> io::Result<()> {
    // SAFETY: a prctl with no pointers. Waits end within a microsecond of
    // when asked, not the default 50.
    unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, 1u64) };
    let clock = Instant::now();
    if let Some(t) = &mut trace {
        t.base_ns = monotonic_ns();
    }
    let seconds = || clock.elapsed().as_secs_f64();
    let fds: Vec<RawFd> = lines.iter().map(|l| l.socket.as_raw_fd()).chain(serial.iter().map(|s| s.pty.fd())).collect();
    let mut buses: Vec<Timeline> = lines.iter().map(|l| Timeline::new(l.rates)).collect();
    let mut wires: Vec<serial_timing::Line> = serial.iter().map(|s| serial_timing::Line::new(s.baud)).collect();
    let mut said = Said::default();
    let mut buf = [0u8; 4096];
    let dt = dock.timestep();
    let (mut start, mut steps) = (0.0, 0u64);
    loop {
        let due = start + dt * (steps + 1) as f64;
        loop {
            let now = seconds();
            for (b, bus) in buses.iter_mut().enumerate() {
                loop {
                    let arrived = bus.advance(now);
                    if arrived.is_empty() {
                        break;
                    }
                    for crossed in arrived {
                        if let Some(t) = &mut trace {
                            t.write(&lines[b].name, &crossed)?;
                        }
                        match crossed.from {
                            From::Master => {
                                if let Some(answer) = dock.deliver(b, &crossed.frame) {
                                    bus.push(answer, From::Motor, crossed.end + lines[b].reply);
                                }
                            }
                            From::Motor => lines[b].socket.send(&crossed.frame)?,
                        }
                    }
                }
            }
            for (b, wire) in wires.iter_mut().enumerate() {
                loop {
                    let arrived = wire.advance(now);
                    if arrived.is_empty() {
                        break;
                    }
                    for crossed in arrived {
                        if let Some(t) = &mut trace {
                            t.write_serial(&serial[b].name(), &crossed)?;
                        }
                        match crossed.from {
                            serial_timing::From::Master => {
                                let line = &mut serial[b];
                                for packet in line.packets(&crossed.bytes, crossed.start, crossed.end, &mut said) {
                                    for reply in dock.deliver_packet(b, &packet) {
                                        wire.push(reply.bytes, serial_timing::From::Device, crossed.end + reply.after);
                                    }
                                }
                            }
                            serial_timing::From::Device => serial[b].pty.write(&crossed.bytes)?,
                        }
                    }
                }
            }
            if now >= due {
                break;
            }
            let wake = buses
                .iter()
                .filter_map(Timeline::next_event)
                .chain(wires.iter().filter_map(serial_timing::Line::next_event))
                .fold(due, f64::min);
            let ready = wait_any(&fds, Some(Duration::from_secs_f64((wake - now).max(0.0))))?;
            for (i, _) in ready.iter().enumerate().filter(|(_, &r)| r) {
                if i < lines.len() {
                    while let Some(frame) = lines[i].socket.try_recv()? {
                        buses[i].push(frame, From::Master, seconds());
                    }
                } else {
                    let b = i - lines.len();
                    let n = serial[b].pty.read(&mut buf)?;
                    wires[b].push(buf[..n].to_vec(), serial_timing::From::Master, seconds());
                }
            }
        }
        dock.step();
        steps += 1;
        let now = seconds();
        report(&mut dock, trace.as_mut(), now)?;
        if let Some(t) = &mut trace {
            t.flush_now_and_then(now)?;
        }
        for (bus, line) in buses.iter_mut().zip(lines) {
            if let Some(report) = bus.report(now) {
                eprintln!("drydock: {}: {report}", line.name);
            }
        }
        for (wire, line) in wires.iter_mut().zip(serial.iter()) {
            if let Some(report) = wire.report(now) {
                eprintln!("drydock: {}: {report}", line.name());
            }
        }
        let late = now - due;
        if late > MAX_LAG.as_secs_f64() {
            eprintln!(
                "drydock: {:.0} ms behind the wall clock at t = {:.3} s: catching up from here",
                late * 1e3,
                dock.time()
            );
            (start, steps) = (now, 0);
        }
    }
}

/// When physics runs in lockstep: `period` seconds once every motor has had
/// a frame or a packet, a cycle of the bus masters.
///
/// A cycle a read ends runs just before the next read, so whatever the
/// master sends in between acts during it: a master that reads, then
/// writes, as Pollen's runtime does, reads next what its writes did, as on
/// a real bus, where commands act as soon as they arrive. A cycle a write
/// ends runs at once, so a master that only writes moves time on. Every
/// CAN frame reads: Damiao's motors answer each one with their state.
pub struct Lockstep {
    steps: u32,
    /// A cycle ended by a read, whose physics hasn't run yet.
    owed: bool,
}

impl Lockstep {
    pub fn new(timestep: f64, period: f64) -> Result<Self, String> {
        Ok(Self { steps: steps_per_period(timestep, period)?, owed: false })
    }

    /// Delivers a frame or a packet with `deliver`, which `reads` or not,
    /// running physics before or after it as the cycle says. Its answer,
    /// and whether physics ran.
    pub fn deliver<T>(
        &mut self,
        dock: &mut Drydock,
        reads: bool,
        deliver: impl FnOnce(&mut Drydock) -> T,
    ) -> (T, bool) {
        let mut ran = false;
        if reads && self.owed {
            self.run(dock);
            ran = true;
        }
        let answer = deliver(dock);
        if dock.cycle_done() {
            if reads {
                self.owed = true;
            } else {
                self.run(dock);
                ran = true;
            }
        }
        (answer, ran)
    }

    fn run(&mut self, dock: &mut Drydock) {
        self.owed = false;
        for _ in 0..self.steps {
            dock.step();
        }
    }
}

/// In lockstep with the bus masters, as [`Lockstep`] says: each frame or
/// packet reaches the motors at once and is answered at once, with the
/// state as it is. Time stands still while the masters don't send, and
/// frames and bytes take no time. Runs are the same each time.
pub fn lockstep(mut dock: Drydock, lines: &[Line], serial: &mut [SerialLine], period: f64) -> io::Result<()> {
    let mut clock = Lockstep::new(dock.timestep(), period).map_err(io::Error::other)?;
    let fds: Vec<RawFd> = lines.iter().map(|l| l.socket.as_raw_fd()).chain(serial.iter().map(|s| s.pty.fd())).collect();
    let mut said = Said::default();
    let mut buf = [0u8; 4096];
    loop {
        let ready = wait_any(&fds, None)?;
        for (i, _) in ready.iter().enumerate().filter(|(_, &r)| r) {
            if i < lines.len() {
                while let Some(frame) = lines[i].socket.try_recv()? {
                    let (answer, ran) = clock.deliver(&mut dock, true, |dock| dock.deliver(i, &frame));
                    if ran {
                        report(&mut dock, None, 0.0)?;
                    }
                    if let Some(answer) = answer {
                        lines[i].socket.send(&answer)?;
                    }
                }
            } else {
                let b = i - lines.len();
                let n = serial[b].pty.read(&mut buf)?;
                // No time passes in lockstep: no gap ever drops a packet.
                for packet in serial[b].packets(&buf[..n], 0.0, 0.0, &mut said) {
                    let (replies, ran) =
                        clock.deliver(&mut dock, packet.reads(), |dock| dock.deliver_packet(b, &packet));
                    if ran {
                        report(&mut dock, None, 0.0)?;
                    }
                    for reply in replies {
                        serial[b].pty.write(&reply.bytes)?;
                    }
                }
            }
        }
    }
}

/// How many timesteps make a period, which must be a whole number of them.
pub fn steps_per_period(timestep: f64, period: f64) -> Result<u32, String> {
    let n = (period / timestep).round();
    if n < 1.0 || ((n * timestep - period) / period).abs() > 1e-9 {
        return Err(format!(
            "a lockstep period of {} ms isn't a whole number of the scene's {} ms timesteps",
            period * 1e3,
            timestep * 1e3
        ));
    }
    Ok(n as u32)
}
