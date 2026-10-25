//! Waiting on several buses at once: CAN sockets and serial ports alike.

use std::io;
use std::os::fd::RawFd;
use std::time::Duration;

/// Waits at most `timeout` (`None`: as long as it takes) for any of `fds`
/// to have something to read, and says which do. An interrupted wait ends
/// early, with none.
pub fn wait_any(fds: &[RawFd], timeout: Option<Duration>) -> io::Result<Vec<bool>> {
    let mut polled: Vec<libc::pollfd> =
        fds.iter().map(|&fd| libc::pollfd { fd, events: libc::POLLIN, revents: 0 }).collect();
    let ts = timeout.map(|t| libc::timespec { tv_sec: t.as_secs() as _, tv_nsec: t.subsec_nanos() as _ });
    let ts_ptr = ts.as_ref().map_or(std::ptr::null(), |t| t as *const libc::timespec);
    // SAFETY: `polled` and the optional timespec live through the call.
    let ready = unsafe { libc::ppoll(polled.as_mut_ptr(), polled.len() as _, ts_ptr, std::ptr::null()) };
    if ready < 0 {
        let e = io::Error::last_os_error();
        if e.kind() != io::ErrorKind::Interrupted {
            return Err(e);
        }
        return Ok(vec![false; fds.len()]);
    }
    Ok(polled.iter().map(|f| f.revents & libc::POLLIN != 0).collect())
}
