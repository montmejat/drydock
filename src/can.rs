//! A CAN bus through the kernel's SocketCAN: a raw socket on an interface
//! (`vcan0`, or a real `can0`), exchanging classic frames.

use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd};
use std::time::Duration;

/// A classic CAN frame with an 11-bit id.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Frame {
    pub id: u16,
    pub len: u8,
    pub data: [u8; 8],
}

impl Frame {
    pub fn new(id: u16, data: [u8; 8]) -> Self {
        Self { id, len: 8, data }
    }
}

/// The kernel's `struct can_frame`.
#[repr(C)]
#[derive(Default)]
struct RawFrame {
    id: u32,
    len: u8,
    _pad: [u8; 3],
    data: [u8; 8],
}

const RAW_LEN: usize = std::mem::size_of::<RawFrame>();
/// Extended ids, remote requests and error frames: none of them ours.
const NOT_CLASSIC_DATA: u32 = 0x8000_0000 | 0x4000_0000 | 0x2000_0000;

pub struct Bus {
    socket: OwnedFd,
}

impl Bus {
    pub fn open(interface: &str) -> io::Result<Self> {
        let name = CString::new(interface)?;
        // SAFETY: plain syscalls on a socket we own, with an address that
        // lives through the call.
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
            let mut address: libc::sockaddr_can = std::mem::zeroed();
            address.can_family = libc::AF_CAN as _;
            address.can_ifindex = index as _;
            let len = std::mem::size_of::<libc::sockaddr_can>() as libc::socklen_t;
            if libc::bind(fd, (&raw const address).cast(), len) < 0 {
                return Err(io::Error::last_os_error());
            }
            Ok(Self { socket })
        }
    }

    pub fn send(&self, frame: &Frame) -> io::Result<()> {
        let raw = RawFrame { id: frame.id as u32, len: frame.len.min(8), data: frame.data, ..Default::default() };
        // SAFETY: writes `RAW_LEN` bytes from a live `RawFrame`.
        let n = unsafe { libc::write(self.socket.as_raw_fd(), (&raw const raw).cast(), RAW_LEN) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }

    /// The next frame, waiting at most `timeout` (`None`: as long as it
    /// takes). `None` when nothing came, or when what came isn't a classic
    /// data frame, or the wait was interrupted: callers look at the clock
    /// and ask again.
    pub fn recv(&self, timeout: Option<Duration>) -> io::Result<Option<Frame>> {
        let mut fds = libc::pollfd { fd: self.socket.as_raw_fd(), events: libc::POLLIN, revents: 0 };
        let ts = timeout.map(|t| libc::timespec { tv_sec: t.as_secs() as _, tv_nsec: t.subsec_nanos() as _ });
        let ts_ptr = ts.as_ref().map_or(std::ptr::null(), |t| t as *const libc::timespec);
        // SAFETY: one pollfd and an optional timespec, both live through the call.
        let ready = unsafe { libc::ppoll(&mut fds, 1, ts_ptr, std::ptr::null()) };
        if ready < 0 {
            let e = io::Error::last_os_error();
            return if e.kind() == io::ErrorKind::Interrupted { Ok(None) } else { Err(e) };
        }
        if ready == 0 {
            return Ok(None);
        }
        let mut raw = RawFrame::default();
        // SAFETY: reads at most `RAW_LEN` bytes into a live `RawFrame`.
        let n = unsafe { libc::read(self.socket.as_raw_fd(), (&raw mut raw).cast(), RAW_LEN) };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        if n as usize != RAW_LEN || raw.id & NOT_CLASSIC_DATA != 0 {
            return Ok(None);
        }
        Ok(Some(Frame { id: raw.id as u16, len: raw.len.min(8), data: raw.data }))
    }
}
