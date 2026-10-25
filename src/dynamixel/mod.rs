//! Dynamixel servos on a half-duplex serial bus, speaking Protocol 2.0:
//! its packets (`protocol`), the XL330's firmware (`xl330`), and the
//! servos on one bus (`bus`).

pub mod bus;
pub mod imu;
pub mod protocol;
pub mod xl330;

pub use protocol::*;
