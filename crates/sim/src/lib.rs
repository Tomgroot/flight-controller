//! Quadcopter physics and sensor models. World frame is z-up, body frame is FLU.

mod imu;
mod quad;
mod rc;

pub use imu::{Imu, ImuParams};
pub use quad::{Quad, QuadParams};
pub use rc::RcSource;

pub const GRAVITY: f32 = 9.81;
