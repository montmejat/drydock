//! drydock: a robot's motors on virtual CAN buses, their physics simulated
//! in MuJoCo. A bus master talks to it as it would to the real motors.
//!
//! The pieces, from the bus to the world:
//!
//! - `can`: frames, sockets, and how long frames take on a real bus;
//! - `run`: the clock, real time or lockstep, over every bus;
//! - `damiao`: Damiao's protocol, one motor's firmware, the motors on a bus;
//! - `drivetrain`: what limits a motor's torque, and how it moves its joint;
//! - `thermal`: how hot a motor's coil and drive get;
//! - `effects`: an imperfect encoder, and how a motor breaks;
//! - `scenario`: when motors break, at set times or at random;
//! - `world`: MuJoCo;
//! - `dock`: the robot, putting them together;
//! - `robot`: the robot file.

pub mod can;
pub mod damiao;
pub mod dock;
pub mod drivetrain;
pub mod effects;
pub mod robot;
pub mod run;
pub mod scenario;
pub mod thermal;
pub mod world;
