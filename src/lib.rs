//! drydock: a robot's motors on a virtual CAN bus, their physics simulated in
//! MuJoCo. A bus master talks to it as it would to the real motors.

pub mod can;
pub mod damiao;
pub mod dock;
pub mod motor;
pub mod robot;
pub mod run;
pub mod world;
