//! Flight controller core. Runs unchanged on the STM32F401 and in SITL.
//!
//! Frames: body is FLU (x forward, y left, z up).
#![no_std]

/// One MPU-6050 sample in the body frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuSample {
    /// Angular rate in rad/s.
    pub gyro: [f32; 3],
    /// Specific force in m/s² (reads +9.81 on z when level and at rest).
    pub accel: [f32; 3],
}

pub const RC_MIN: u16 = 172;
pub const RC_MID: u16 = 992;
pub const RC_MAX: u16 = 1811;

/// CRSF channels (172..=1811). AETR order: 0 roll, 1 pitch, 2 throttle, 3 yaw, 4 arm switch.
#[derive(Clone, Copy, Debug)]
pub struct RcChannels(pub [u16; 16]);

impl Default for RcChannels {
    fn default() -> Self {
        let mut ch = [RC_MID; 16];
        ch[2] = RC_MIN;
        ch[4] = RC_MIN;
        RcChannels(ch)
    }
}

/// Motor commands 0.0..=1.0, Betaflight quad-X order:
/// 0 rear-right (CW), 1 front-right (CCW), 2 rear-left (CCW), 3 front-left (CW).
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorOutputs(pub [f32; 4]);

pub struct FlightController {
    // TODO: estimator state, PID state, armed flag, ...
}

impl FlightController {
    pub fn new() -> Self {
        FlightController {}
    }

    pub fn update(&mut self, imu: &ImuSample, rc: &RcChannels, dt: f32) -> MotorOutputs {
        let _ = (imu, rc, dt);
        // TODO: arming logic (rc.0[4])
        // TODO: attitude estimation (complementary / Mahony filter on imu)
        // TODO: angle -> rate setpoints from sticks
        // TODO: rate PID
        // TODO: mixer -> MotorOutputs
        MotorOutputs::default()
    }
}

impl Default for FlightController {
    fn default() -> Self {
        Self::new()
    }
}
