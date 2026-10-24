//! CAN: frames, a SocketCAN socket to carry them (`vcan0`, or a real
//! `can0`), and how long they take on a real bus (`timing`).
//!
//! CAN FD frames carry up to 64 bytes, and switch to a faster bit rate for
//! their data (BRS); classic frames carry up to 8. Damiao's motors use 8
//! either way.

pub mod timing;

use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

/// A frame with an 11-bit id: classic, or CAN FD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    pub id: u16,
    pub len: u8,
    pub data: [u8; 64],
    /// A CAN FD frame, with its data at the faster bit rate.
    pub fd: bool,
}

impl Frame {
    /// A classic frame of 8 bytes.
    pub fn new(id: u16, data: [u8; 8]) -> Self {
        let mut all = [0; 64];
        all[..8].copy_from_slice(&data);
        Self { id, len: 8, data: all, fd: false }
    }

    /// The same frame, as CAN FD or classic.
    pub fn fd(self, fd: bool) -> Self {
        Self { fd, ..self }
    }

    /// The data, when it is 8 bytes long, as every Damiao frame is.
    pub fn eight(&self) -> Option<[u8; 8]> {
        (self.len == 8).then(|| self.data[..8].try_into().unwrap())
    }
}

/// The kernel's `struct canfd_frame`; a `struct can_frame` is its first 16
/// bytes, with `flags` unused.
#[repr(C)]
struct RawFrame {
    id: u32,
    len: u8,
    flags: u8,
    _reserved: [u8; 2],
    data: [u8; 64],
}

const CAN_MTU: usize = 16;
const CANFD_MTU: usize = std::mem::size_of::<RawFrame>();
/// The data phase at the faster bit rate.
const CANFD_BRS: u8 = 0x01;
/// Extended ids, remote requests and error frames: none of them ours.
const NOT_DATA_FRAME: u32 = 0x8000_0000 | 0x4000_0000 | 0x2000_0000;
const SOL_CAN_RAW: libc::c_int = libc::SOL_CAN_BASE + libc::CAN_RAW;
const CAN_RAW_FD_FRAMES: libc::c_int = 5;

/// A raw CAN socket on one interface.
pub struct Socket {
    socket: OwnedFd,
    /// Whether the interface carries CAN FD frames (its MTU is 72).
    fd_capable: bool,
}

impl Socket {
    pub fn open(interface: &str) -> io::Result<Self> {
        let name = CString::new(interface)?;
        // SAFETY: plain syscalls on a socket we own, with arguments that
        // live through each call.
        unsafe {
            let index = libc::if_nametoindex(name.as_ptr());
            if index == 0 {
                return Err(io::Error::other(format!("no CAN interface `{interface}`")));
            }
            let fd = libc::socket(libc::PF_CAN, libc::SOCK_RAW | libc::SOCK_CLOEXEC, libc::CAN_RAW);
            if fd < 0 {
                return Err(io::Error::last_os_error());
            }
            let socket = OwnedFd::from_raw_fd(fd);
            // Both kinds in: an FD frame on a classic robot's bus is worth
            // noticing, not missing.
            let on: libc::c_int = 1;
            let size = std::mem::size_of::<libc::c_int>() as libc::socklen_t;
            if libc::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, (&raw const on).cast(), size) < 0 {
                return Err(io::Error::last_os_error());
            }
            let mut ifr: libc::ifreq = std::mem::zeroed();
            for (dst, &src) in ifr.ifr_name.iter_mut().zip(name.as_bytes()) {
                *dst = src as libc::c_char;
            }
            if libc::ioctl(fd, libc::SIOCGIFMTU, &mut ifr) < 0 {
                return Err(io::Error::last_os_error());
            }
            let fd_capable = ifr.ifr_ifru.ifru_mtu as usize == CANFD_MTU;
            let mut address: libc::sockaddr_can = std::mem::zeroed();
            address.can_family = libc::AF_CAN as _;
            address.can_ifindex = index as _;
            let len = std::mem::size_of::<libc::sockaddr_can>() as libc::socklen_t;
            if libc::bind(fd, (&raw const address).cast(), len) < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(Self { socket, fd_capable })
        }
    }

    /// Whether the interface carries CAN FD frames.
    pub fn fd_capable(&self) -> bool {
        self.fd_capable
    }

    pub fn send(&self, frame: &Frame) -> io::Result<()> {
        let (len, size, flags) = match frame.fd {
            true => (frame.len.min(64), CANFD_MTU, CANFD_BRS),
            false => (frame.len.min(8), CAN_MTU, 0),
        };
        let raw = RawFrame { id: frame.id as u32, len, flags, _reserved: [0; 2], data: frame.data };
        // SAFETY: writes `size` bytes, at most all of a live `RawFrame`.
        let n = unsafe { libc::write(self.socket.as_raw_fd(), (&raw const raw).cast(), size) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// The next frame if one is waiting, without waiting for one. What
    /// isn't a data frame with an 11-bit id is passed over.
    pub fn try_recv(&self) -> io::Result<Option<Frame>> {
        loop {
            let mut raw = RawFrame { id: 0, len: 0, flags: 0, _reserved: [0; 2], data: [0; 64] };
            // SAFETY: reads at most `CANFD_MTU` bytes into a live `RawFrame`.
            let n =
                unsafe { libc::recv(self.socket.as_raw_fd(), (&raw mut raw).cast(), CANFD_MTU, libc::MSG_DONTWAIT) };
            if n < 0 {
                let e = io::Error::last_os_error();
                return match e.kind() {
                    io::ErrorKind::WouldBlock => Ok(None),
                    io::ErrorKind::Interrupted => continue,
                    _ => Err(e),
                };
            }
            let fd = match n as usize {
                CAN_MTU => false,
                CANFD_MTU => true,
                _ => continue,
            };
            if raw.id & NOT_DATA_FRAME != 0 {
                continue;
            }
            let len = raw.len.min(if fd { 64 } else { 8 });
            return Ok(Some(Frame { id: raw.id as u16, len, data: raw.data, fd }));
        }
    }

    /// The next frame, waiting at most `timeout` (`None`: as long as it
    /// takes). `None` if none came in time.
    pub fn recv(&self, timeout: Option<Duration>) -> io::Result<Option<Frame>> {
        if wait_any(&[self], timeout)?.first() == Some(&true) {
            return self.try_recv();
        }
        Ok(None)
    }
}

/// Waits at most `timeout` (`None`: as long as it takes) for any of
/// `sockets` to have a frame, and says which do. An interrupted wait ends
/// early, with none.
pub fn wait_any(sockets: &[&Socket], timeout: Option<Duration>) -> io::Result<Vec<bool>> {
    let mut fds: Vec<libc::pollfd> =
        sockets.iter().map(|s| libc::pollfd { fd: s.socket.as_raw_fd(), events: libc::POLLIN, revents: 0 }).collect();
    let ts = timeout.map(|t| libc::timespec { tv_sec: t.as_secs() as _, tv_nsec: t.subsec_nanos() as _ });
    let ts_ptr = ts.as_ref().map_or(std::ptr::null(), |t| t as *const libc::timespec);
    // SAFETY: `fds` and the optional timespec live through the call.
    let ready = unsafe { libc::ppoll(fds.as_mut_ptr(), fds.len() as _, ts_ptr, std::ptr::null()) };
    if ready < 0 {
        let e = io::Error::last_os_error();
        if e.kind() != io::ErrorKind::Interrupted {
            return Err(e);
        }
        return Ok(vec![false; sockets.len()]);
    }
    Ok(fds.iter().map(|f| f.revents & libc::POLLIN != 0).collect())
}
