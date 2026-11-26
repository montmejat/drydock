//! EtherCAT: CiA 402 servo drives, each on its own EtherCAT chip, in a line
//! on an Ethernet interface (`ecs0`, one end of a veth pair whose other end
//! the master opens).
//!
//! From the wire up:
//!
//! - `frame`: an EtherCAT frame's datagrams, and the line of devices one
//!   passes through;
//! - `esc`: the chip, a Beckhoff ET1100: its registers, its SII EEPROM
//!   interface, its sync managers and FMMUs, as the master sees them;
//! - `sii`: what that EEPROM holds, the device's identity and layout;
//! - `coe`: CANopen over EtherCAT: the mailbox's SDO transfers;
//! - `drive`: the drive's firmware: its states, its object dictionary, its
//!   PDOs, and CiA 402's state machine and modes.
//!
//! A frame passes through the drives in the robot file's order, as through
//! a cable, and comes back the way it went: the last drive's second port is
//! open, so the frame turns there. Each chip reads and writes its part of
//! each datagram on the way out; on the way back, the frame only passes.

pub mod coe;
pub mod drive;
pub mod esc;
pub mod frame;
pub mod sii;

use std::ffi::CString;
use std::io;
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};

/// EtherCAT's EtherType.
pub const ETHERTYPE: u16 = 0x88A4;

/// A raw socket on one interface, for EtherCAT frames only.
pub struct Socket {
    fd: OwnedFd,
    ifindex: i32,
}

impl Socket {
    /// Opens `interface`. It needs CAP_NET_RAW: root, or a network
    /// namespace of one's own (`unshare -rn`).
    pub fn open(interface: &str) -> io::Result<Self> {
        let name = CString::new(interface).map_err(|_| io::Error::other("an interface name has no NUL"))?;
        // SAFETY: a C string that lives through the call.
        let ifindex = unsafe { libc::if_nametoindex(name.as_ptr()) } as i32;
        if ifindex == 0 {
            return Err(io::Error::last_os_error());
        }
        let protocol = (ETHERTYPE).to_be() as i32;
        // SAFETY: plain syscall; the fd is owned from here on.
        let raw = unsafe { libc::socket(libc::AF_PACKET, libc::SOCK_RAW | libc::SOCK_NONBLOCK, protocol) };
        if raw < 0 {
            return Err(io::Error::last_os_error());
        }
        // SAFETY: `raw` is a fresh fd nothing else owns.
        let fd = unsafe { OwnedFd::from_raw_fd(raw) };
        // SAFETY: zeroed sockaddr_ll is valid; filled in below.
        let mut addr: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        addr.sll_family = libc::AF_PACKET as u16;
        addr.sll_protocol = (ETHERTYPE).to_be();
        addr.sll_ifindex = ifindex;
        // SAFETY: `addr` lives through the call, with its size.
        let bound = unsafe {
            libc::bind(
                fd.as_raw_fd(),
                &addr as *const libc::sockaddr_ll as *const libc::sockaddr,
                std::mem::size_of::<libc::sockaddr_ll>() as u32,
            )
        };
        if bound < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(Self { fd, ifindex })
    }

    /// A frame the master sent, if one is waiting; frames this socket sent
    /// itself are skipped.
    pub fn try_recv(&self) -> io::Result<Option<Vec<u8>>> {
        let mut buf = vec![0u8; 1536];
        loop {
            // SAFETY: zeroed sockaddr_ll is valid; the kernel fills it in.
            let mut from: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
            let mut len = std::mem::size_of::<libc::sockaddr_ll>() as u32;
            // SAFETY: `buf` and `from` live through the call, with their sizes.
            let n = unsafe {
                libc::recvfrom(
                    self.fd.as_raw_fd(),
                    buf.as_mut_ptr() as *mut libc::c_void,
                    buf.len(),
                    0,
                    &mut from as *mut libc::sockaddr_ll as *mut libc::sockaddr,
                    &mut len,
                )
            };
            if n < 0 {
                let e = io::Error::last_os_error();
                return match e.kind() {
                    io::ErrorKind::WouldBlock => Ok(None),
                    io::ErrorKind::Interrupted => continue,
                    _ => Err(e),
                };
            }
            if from.sll_pkttype == libc::PACKET_OUTGOING {
                continue;
            }
            buf.truncate(n as usize);
            return Ok(Some(buf));
        }
    }

    /// Sends a frame back to the master.
    pub fn send(&self, frame: &[u8]) -> io::Result<()> {
        // SAFETY: zeroed sockaddr_ll is valid; filled in below.
        let mut to: libc::sockaddr_ll = unsafe { std::mem::zeroed() };
        to.sll_family = libc::AF_PACKET as u16;
        to.sll_protocol = (ETHERTYPE).to_be();
        to.sll_ifindex = self.ifindex;
        to.sll_halen = 6;
        to.sll_addr[..6].copy_from_slice(&frame[..6]);
        // SAFETY: `frame` and `to` live through the call, with their sizes.
        let n = unsafe {
            libc::sendto(
                self.fd.as_raw_fd(),
                frame.as_ptr() as *const libc::c_void,
                frame.len(),
                0,
                &to as *const libc::sockaddr_ll as *const libc::sockaddr,
                std::mem::size_of::<libc::sockaddr_ll>() as u32,
            )
        };
        if n < 0 {
            return Err(io::Error::last_os_error());
        }
        Ok(())
    }
}

impl AsRawFd for Socket {
    fn as_raw_fd(&self) -> RawFd {
        self.fd.as_raw_fd()
    }
}
