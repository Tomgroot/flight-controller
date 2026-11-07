//! Flight controller core. Runs unchanged on the STM32F401 and in SITL.
//!
//! Frames: body is FLU (x forward, y left, z up).
#![no_std]

use glam::{Vec2, Vec3};

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
const MAX_ANGLE: f32 = 0.52;
// 1 / 4 = 0.25s to close the gap.
const ANGLE_GAIN: f32 = 4.0;
// 200 degrees per second.
const MAX_YAW_RATE: f32 = 3.5;

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

impl RcChannels {
    pub fn stick(&self, c: Channel) -> f32 {
        ((self[c] as f32 - RC_MID as f32) / (RC_MAX - RC_MID) as f32).clamp(-1.0, 1.0)
    }

    pub fn throttle(&self) -> f32 {
        ((self[Channel::Throttle] as f32 - RC_MIN as f32) / (RC_MAX - RC_MIN) as f32)
            .clamp(0.0, 1.0)
    }
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

const KP: Vec3 = Vec3::new(0.05, 0.05, 0.1);
const KI: Vec3 = Vec3::new(0.0, 0.0, 0.0);
const KD: Vec3 = Vec3::new(0.0, 0.0, 0.0);

pub struct FlightController {
    armed: bool,
    attitude: Vec2,
    integral: Vec3,
    prev_gyro: Vec3,
}

impl FlightController {
    pub fn new() -> Self {
        FlightController {
            armed: false,
            attitude: Vec2::default(),
            integral: Vec3::default(),
            prev_gyro: Vec3::default(),
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
        // Trust gyro by 98%
        const ALPHA: f32 = 0.98;
        // Roll (φ, x-axis) angle, pitch (θ, y-axis) angle
        // ax = −g·sinθ
        // ay =  g·cosθ·sinφ
        // az =  g·cosθ·cosφ
        let accel = Vec2::new(
            libm::atan2f(imu.accel.y, imu.accel.z),
            libm::atan2f(
                -imu.accel.x,
                libm::sqrtf(imu.accel.y * imu.accel.y + imu.accel.z * imu.accel.z),
            ),
        );
        let gyro = self.attitude + imu.gyro.truncate() * dt;
        self.attitude = gyro * ALPHA + accel * (1.0 - ALPHA);

        let angle_setpoint =
            Vec2::new(rc.stick(Channel::Roll), rc.stick(Channel::Pitch)) * MAX_ANGLE;
        let rotate_xy = (angle_setpoint - self.attitude) * ANGLE_GAIN;
        // rad/s
        let rotate_setpoint = rotate_xy.extend(-rc.stick(Channel::Yaw) * MAX_YAW_RATE);
        // P is proportional error correction, I is integral over time correction and D is derivative correction to not overshoot
        let error = rotate_setpoint - imu.gyro;
        self.integral += error * dt;
        let derivative = -(imu.gyro - self.prev_gyro) / dt;
        self.prev_gyro = imu.gyro;

        self.update_arming(rc);
        if !self.armed {
            self.integral = Vec3::ZERO;
            return MotorOutputs::default();
        }

        let output = KP * error + KI * self.integral + KD * derivative;

        let t = rc.throttle();
        MotorOutputs(
            [
                t - output.x + output.y + output.z, // RearRight (CW)
                t - output.x - output.y - output.z, // FrontRight (CCW)
                t + output.x + output.y - output.z, // RearLeft (CCW)
                t + output.x - output.y + output.z, // FrontLeft (CW)
            ]
            .map(|m| m.clamp(0.0, 1.0)),
        )
    }
}

impl Default for FlightController {
    fn default() -> Self {
        Self::new()
    }
}
