//! Damiao's motors in MIT mode: their frames (`protocol`), one motor's
//! firmware (`motor`), and the motors on one CAN bus (`bus`).

pub mod bus;
pub mod motor;
pub mod protocol;

pub use protocol::*;
