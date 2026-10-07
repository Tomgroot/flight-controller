//! Flight controller core. Runs unchanged on the STM32F401 and in SITL.
//!
//! Frames: body is FLU (x forward, y left, z up).
#![no_std]

use glam::Vec3;

/// One MPU-6050 sample in the body frame.
#[derive(Clone, Copy, Debug, Default)]
pub struct ImuSample {
    /// Angular rate in rad/s.
    pub gyro: Vec3,
    /// Specific force in m/s² (reads +9.81 on z when level and at rest).
    pub accel: Vec3,
}

pub const RC_MIN: u16 = 172;
pub const RC_MID: u16 = 992;
pub const RC_MAX: u16 = 1811;

/// CRSF channels (172..=1811). AETR order: 0 roll, 1 pitch, 2 throttle, 3 yaw, 4 arm switch.
#[derive(Clone, Copy, Debug)]
pub struct RcChannels(pub [u16; 16]);

pub enum Channel {
    Roll,
    Pitch,
    Throttle,
    Yaw,
    Arm,
}

impl core::ops::Index<Channel> for RcChannels {
    type Output = u16;
    fn index(&self, c: Channel) -> &u16 {
        &self.0[c as usize]
    }
}

impl core::ops::IndexMut<Channel> for RcChannels {
    fn index_mut(&mut self, c: Channel) -> &mut u16 {
        &mut self.0[c as usize]
    }
}

impl Default for RcChannels {
    fn default() -> Self {
        let mut ch = RcChannels([RC_MID; 16]);
        ch[Channel::Throttle] = RC_MIN;
        ch[Channel::Arm] = RC_MIN;
        ch
    }
}

/// Motor commands 0.0..=1.0, Betaflight quad-X order:
/// 0 rear-right (CW), 1 front-right (CCW), 2 rear-left (CCW), 3 front-left (CW).
#[derive(Clone, Copy, Debug, Default)]
pub struct MotorOutputs(pub [f32; 4]);

enum Motor {
    RearRight,
    FrontRight,
    RearLeft,
    FrontLeft,
}

pub struct FlightController {
    armed: bool,
    attitude: f32,
}

impl FlightController {
    pub fn new() -> Self {
        FlightController {
            armed: false,
            attitude: 0.0,
        }
    }

    fn update_arming(&mut self, rc: &RcChannels) {
        if rc[Channel::Arm] <= RC_MID {
            self.armed = false;
            return;
        }
        if self.armed {
            return;
        }
        self.armed = rc[Channel::Throttle] <= RC_MIN + 50;
    }

    pub fn update(&mut self, imu: &ImuSample, rc: &RcChannels, dt: f32) -> MotorOutputs {
        self.update_arming(rc);
        if !self.armed {
            return MotorOutputs::default();
        }

        let delta = imu.accel.z - 9.81;
        // dt 0.001 = 1ms, delta m per second, so 10m/s means 10*0.001=0.01m
        let distance = dt * delta;
        self.attitude += distance;

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
