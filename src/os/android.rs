extern crate libc;
extern crate ioctl_rs;

use std::mem;
use libc::{c_int, pid_t, intptr_t, uintptr_t};
use self::ioctl_rs::{ioctl, TCGETS, TCSETS, TCSETSW, TCSETSF, TCSBRKP, TCSBRK, TCFLSH, TCXONC};

pub use super::linux::*;

#[no_mangle]
pub extern "C" fn tcgetattr(fd: c_int, termios_p: *mut ::os::target::termios) -> c_int {
    unsafe { ioctl(fd, TCGETS, termios_p) }
}

#[no_mangle]
pub fn tcsetattr(fd: c_int, optional_actions: c_int, termios_p: *const ::os::target::termios) -> c_int {
    let cmd = match optional_actions {
        TCSANOW     => TCSETS,
        TCSADRAIN   => TCSETSW,
        TCSAFLUSH   => TCSETSF,
        _           => {
            // errno = EINVAL;
            -1
        }
    };
    unsafe { ioctl(fd, cmd, termios_p) }
}

#[no_mangle]
pub fn tcsendbreak(fd: c_int, duration: c_int) -> c_int {
    unsafe { ioctl(fd, TCSBRKP, duration as uintptr_t) }
}

#[no_mangle]
pub extern "C" fn tcdrain(fd: c_int) -> c_int {
    // A non-zero argument to TCSBRK means "don't send a break".
    // The drain is a side-effect of the ioctl!
    unsafe { ioctl(fd, TCSBRK, 1 as intptr_t) }
}

#[no_mangle]
pub extern "C" fn tcflush(fd: c_int, queue_selector: c_int) -> c_int {
    unsafe { ioctl(fd, TCFLSH, queue_selector as intptr_t) }
}

#[no_mangle]
pub extern "C" fn tcflow(fd: c_int, action: c_int) -> c_int {
    unsafe { ioctl(fd, TCXONC, action as intptr_t) }
}

#[no_mangle]
pub extern "C" fn cfmakeraw(termios_p: *mut ::os::target::termios) {
    let mut s = unsafe { *termios_p };
    s.c_iflag &= !(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);
    s.c_oflag &= !OPOST;
    s.c_lflag &= !(ECHO|ECHONL|ICANON|ISIG|IEXTEN);
    s.c_cflag &= !(CSIZE|PARENB);
    s.c_cflag |= CS8;
}

fn cfgetspeed(termios_p: *const ::os::target::termios) -> ::os::target::speed_t {
    let s = unsafe { *termios_p };
    (s.c_cflag & CBAUD)
}

#[no_mangle]
pub extern "C" fn cfgetispeed(termios_p: *const ::os::target::termios) -> ::os::target::speed_t {
    cfgetspeed(termios_p)
}

#[no_mangle]
pub extern "C" fn cfgetospeed(termios_p: *const ::os::target::termios) -> ::os::target::speed_t {
    cfgetspeed(termios_p)
}

#[no_mangle]
pub extern "C" fn cfsetispeed(termios_p: *mut ::os::target::termios, speed: ::os::target::speed_t) -> c_int {
    cfsetspeed(termios_p, speed)
}

#[no_mangle]
pub extern "C" fn cfsetospeed(termios_p: *mut ::os::target::termios, speed: ::os::target::speed_t) -> c_int {
    cfsetspeed(termios_p, speed)
}

#[no_mangle]
pub extern "C" fn cfsetspeed(termios_p: *mut ::os::target::termios, speed: ::os::target::speed_t) -> c_int {
    let mut s = unsafe { *termios_p };
    s.c_cflag = (s.c_cflag & !CBAUD) | (speed & CBAUD);
    0
}

#[cfg(not(target_arch = "mips"))]
const TIOCGSID: c_int = 0x00005429;
#[cfg(target_arch = "mips")]
const TIOCGSID: c_int = 0x00007416;

#[no_mangle]
pub extern "C" fn tcgetsid(fd: c_int) -> pid_t {
    let mut sid: pid_t = unsafe { mem::uninitialized() };
    unsafe {
        if ioctl(fd, TIOCGSID, &mut sid) != -1 {
            sid
        } else {
            -1
        }
    }
}
