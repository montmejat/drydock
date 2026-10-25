//! A serial port, as a pseudo-terminal (pty): drydock holds one end, and
//! links the other at the path the robot file names, for the robot's
//! software to open as it would its real port (`/dev/ttyS2`). A pty takes a
//! baud rate and ignores it: bytes cross at once. How long they'd take is
//! `timing`'s.

pub mod timing;

use std::ffi::CStr;
use std::fs::File;
use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, FromRawFd, OwnedFd, RawFd};
use std::path::{Path, PathBuf};

pub struct Pty {
    master: File,
    /// The far end, held open so the port stays up between the robot's
    /// software closing it and opening it again.
    _slave: OwnedFd,
    link: PathBuf,
}

impl Pty {
    /// A new pty, its far end raw (no echo, no line editing: bytes as they
    /// come), linked at `link`. A link already there, from a run before, is
    /// replaced; anything else there is left alone, with an error.
    pub fn open(link: &Path) -> io::Result<Self> {
        if let Ok(meta) = std::fs::symlink_metadata(link) {
            if !meta.file_type().is_symlink() {
                return Err(io::Error::other(format!(
                    "{} is there already and isn't a link: drydock links its serial port there, so pick another path",
                    link.display()
                )));
            }
            std::fs::remove_file(link)?;
        }
        // SAFETY: plain syscalls on descriptors we own; `ptsname`'s buffer
        // is copied before the next call.
        unsafe {
            let fd = libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC);
            if fd < 0 || libc::grantpt(fd) != 0 || libc::unlockpt(fd) != 0 {
                return Err(io::Error::last_os_error());
            }
            let master = File::from_raw_fd(fd);
            let name = CStr::from_ptr(libc::ptsname(fd)).to_owned();
            let slave = libc::open(name.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC);
            if slave < 0 {
                return Err(io::Error::last_os_error());
            }
            let slave = OwnedFd::from_raw_fd(slave);
            let mut t: libc::termios = std::mem::zeroed();
            if libc::tcgetattr(slave.as_raw_fd(), &mut t) != 0 {
                return Err(io::Error::last_os_error());
            }
            libc::cfmakeraw(&mut t);
            if libc::tcsetattr(slave.as_raw_fd(), libc::TCSANOW, &t) != 0 {
                return Err(io::Error::last_os_error());
            }
            let target = PathBuf::from(name.to_string_lossy().into_owned());
            std::os::unix::fs::symlink(&target, link)?;
            Ok(Self { master, _slave: slave, link: link.to_path_buf() })
        }
    }

    /// Where the robot's software opens it.
    pub fn link(&self) -> &Path {
        &self.link
    }

    /// What the robot's software has written since, at most `buf`'s length;
    /// none if nothing.
    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.master.read(buf) {
            Ok(n) => Ok(n),
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(0),
            Err(e) => Err(e),
        }
    }

    /// Bytes to the robot's software. If it isn't reading and the pty's
    /// buffer is full, what doesn't fit is lost, as on a real line.
    pub fn write(&mut self, bytes: &[u8]) -> io::Result<()> {
        match self.master.write_all(bytes) {
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => Ok(()),
            other => other,
        }
    }

    pub fn fd(&self) -> RawFd {
        self.master.as_raw_fd()
    }
}

impl Drop for Pty {
    fn drop(&mut self) {
        let _ = std::fs::remove_file(&self.link);
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bytes_cross_both_ways_and_the_link_goes_with_it() {
        let link = std::env::temp_dir().join(format!("drydock-test-tty-{}", std::process::id()));
        let mut pty = Pty::open(&link).unwrap();
        let mut robot = std::fs::OpenOptions::new().read(true).write(true).open(&link).unwrap();
        robot.write_all(&[0xFF, 0xFF, 0xFD, 0x00]).unwrap();
        let mut buf = [0u8; 16];
        std::thread::sleep(std::time::Duration::from_millis(10));
        assert_eq!(pty.read(&mut buf).unwrap(), 4);
        assert_eq!(buf[..4], [0xFF, 0xFF, 0xFD, 0x00], "raw: bytes unchanged");
        pty.write(&[0x55, 0xAA]).unwrap();
        let mut back = [0u8; 2];
        robot.read_exact(&mut back).unwrap();
        assert_eq!(back, [0x55, 0xAA]);
        assert_eq!(pty.read(&mut buf).unwrap(), 0, "no echo of what drydock wrote");
        drop(pty);
        assert!(std::fs::symlink_metadata(&link).is_err());
    }
}
