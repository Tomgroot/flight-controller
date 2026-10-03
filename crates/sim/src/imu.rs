use crate::{Quad, GRAVITY};
use fc_core::ImuSample;
use glam::Vec3;

/// MPU-6050 model: range limits, white noise and a constant bias.
#[derive(Clone, Debug)]
pub struct ImuParams {
    /// rad/s (±2000 °/s).
    pub gyro_range: f32,
    /// m/s² (±8 g).
    pub accel_range: f32,
    /// rad/s, standard deviation.
    pub gyro_noise: f32,
    /// m/s², standard deviation.
    pub accel_noise: f32,
    pub gyro_bias: Vec3,
    pub accel_bias: Vec3,
}

impl Default for ImuParams {
    fn default() -> Self {
        ImuParams {
            gyro_range: 2000f32.to_radians(),
            accel_range: 8.0 * GRAVITY,
            gyro_noise: 0.005,
            accel_noise: 0.04,
            gyro_bias: Vec3::new(0.01, -0.008, 0.005),
            accel_bias: Vec3::new(0.05, -0.03, 0.08),
        }
    }
}

pub struct Imu {
    pub params: ImuParams,
    rng: u64,
}

impl Imu {
    pub fn new(params: ImuParams, seed: u64) -> Self {
        Imu {
            params,
            rng: seed.max(1),
        }
    }

    pub fn sample(&mut self, quad: &Quad) -> ImuSample {
        let p = self.params.clone();
        let gyro = quad.angular_velocity + p.gyro_bias + self.noise() * p.gyro_noise;
        let accel = quad.specific_force() + p.accel_bias + self.noise() * p.accel_noise;
        ImuSample {
            gyro: gyro.clamp(Vec3::splat(-p.gyro_range), Vec3::splat(p.gyro_range)).to_array(),
            accel: accel.clamp(Vec3::splat(-p.accel_range), Vec3::splat(p.accel_range)).to_array(),
        }
    }

    /// Three independent standard normal samples.
    fn noise(&mut self) -> Vec3 {
        Vec3::new(self.gaussian(), self.gaussian(), self.gaussian())
    }

    /// Box-Muller over xorshift64, deterministic for a given seed.
    fn gaussian(&mut self) -> f32 {
        let u1 = self.uniform().max(f32::MIN_POSITIVE);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (core::f32::consts::TAU * u2).cos()
    }

    fn uniform(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 40) as f32 / (1u64 << 24) as f32
    }
}
