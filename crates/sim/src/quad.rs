use crate::GRAVITY;
use fc_core::MotorOutputs;
use glam::{Quat, Vec3};

/// Airframe parameters. Defaults approximate PARTS.md:
/// Mark4 10" frame, SUNNYSKY X2212 980KV on 3S, 9047 props.
#[derive(Clone, Debug)]
pub struct QuadParams {
    /// kg (estimate, weigh the real build and update).
    pub mass: f32,
    /// kg·m², body-frame diagonal inertia.
    pub inertia: Vec3,
    /// m, center to motor (Mark4 10" ≈ 450 mm wheelbase).
    pub arm_length: f32,
    /// rad/s at full throttle (980 KV × 11.1 V ≈ 10.9k rpm).
    pub max_motor_speed: f32,
    /// s, first-order motor spin-up time constant.
    pub motor_tau: f32,
    /// N/(rad/s)², thrust = k_thrust·ω².
    pub k_thrust: f32,
    /// N·m/(rad/s)², yaw reaction torque = k_torque·ω².
    pub k_torque: f32,
    /// N/(m/s), linear air drag.
    pub linear_drag: f32,
    /// N·m/(rad/s), rotational damping.
    pub angular_drag: f32,
}

impl Default for QuadParams {
    fn default() -> Self {
        let max_motor_speed = 980.0 * 11.1 * core::f32::consts::TAU / 60.0;
        // ≈ 850 g of thrust per motor at full throttle.
        let k_thrust = 0.85 * GRAVITY / (max_motor_speed * max_motor_speed);
        QuadParams {
            mass: 1.0,
            inertia: Vec3::new(0.012, 0.012, 0.022),
            arm_length: 0.225,
            max_motor_speed,
            motor_tau: 0.05,
            k_thrust,
            k_torque: k_thrust * 0.016,
            linear_drag: 0.1,
            angular_drag: 0.002,
        }
    }
}

impl QuadParams {
    /// Motor positions in body frame, `MotorOutputs` order.
    pub fn motor_positions(&self) -> [Vec3; 4] {
        let d = self.arm_length * core::f32::consts::FRAC_1_SQRT_2;
        [
            Vec3::new(-d, -d, 0.0), // rear-right
            Vec3::new(d, -d, 0.0),  // front-right
            Vec3::new(-d, d, 0.0),  // rear-left
            Vec3::new(d, d, 0.0),   // front-left
        ]
    }

    /// +1 for CW props (seen from above), -1 for CCW. A CW prop yaws the body CCW (+z).
    pub fn motor_spin(&self) -> [f32; 4] {
        [1.0, -1.0, -1.0, 1.0]
    }

    /// Throttle at which all four motors together carry the quad's weight.
    pub fn hover_throttle(&self) -> f32 {
        (self.mass * GRAVITY / (4.0 * self.k_thrust)).sqrt() / self.max_motor_speed
    }
}

#[derive(Clone, Debug)]
pub struct Quad {
    pub params: QuadParams,
    /// World position, m.
    pub position: Vec3,
    /// World velocity, m/s.
    pub velocity: Vec3,
    /// Body to world rotation.
    pub attitude: Quat,
    /// Body angular rate, rad/s.
    pub angular_velocity: Vec3,
    /// rad/s per motor.
    pub motor_speeds: [f32; 4],
    /// World acceleration of the last step, m/s².
    pub acceleration: Vec3,
}

impl Quad {
    pub fn new(params: QuadParams) -> Self {
        Quad {
            params,
            position: Vec3::ZERO,
            velocity: Vec3::ZERO,
            attitude: Quat::IDENTITY,
            angular_velocity: Vec3::ZERO,
            motor_speeds: [0.0; 4],
            acceleration: Vec3::ZERO,
        }
    }

    /// Specific force in body frame, what an ideal accelerometer measures.
    pub fn specific_force(&self) -> Vec3 {
        self.attitude.inverse() * (self.acceleration + Vec3::Z * GRAVITY)
    }

    pub fn on_ground(&self) -> bool {
        self.position.z <= 0.0
    }

    pub fn step(&mut self, dt: f32, motors: &MotorOutputs) {
        let p = &self.params;
        let positions = p.motor_positions();
        let spin = p.motor_spin();

        let mut thrust = 0.0;
        let mut torque = Vec3::ZERO;
        for i in 0..4 {
            let cmd = motors.0[i].clamp(0.0, 1.0) * p.max_motor_speed;
            self.motor_speeds[i] += (cmd - self.motor_speeds[i]) * (dt / p.motor_tau).min(1.0);
            let w2 = self.motor_speeds[i] * self.motor_speeds[i];
            let f = p.k_thrust * w2;
            thrust += f;
            torque += positions[i].cross(Vec3::Z * f);
            torque.z += spin[i] * p.k_torque * w2;
        }

        let force = self.attitude * (Vec3::Z * thrust) - Vec3::Z * p.mass * GRAVITY
            - self.velocity * p.linear_drag;
        self.acceleration = force / p.mass;

        let w = self.angular_velocity;
        torque -= w * p.angular_drag;
        let angular_acceleration = (torque - w.cross(p.inertia * w)) / p.inertia;

        // Resting on the ground: contact cancels any net downward force.
        if self.on_ground() && self.acceleration.z <= 0.0 {
            self.position.z = 0.0;
            self.velocity = Vec3::ZERO;
            self.acceleration = Vec3::ZERO;
            self.angular_velocity = Vec3::ZERO;
            return;
        }

        // Semi-implicit Euler.
        self.velocity += self.acceleration * dt;
        self.position += self.velocity * dt;
        self.angular_velocity += angular_acceleration * dt;
        let w = self.angular_velocity;
        let dq = Quat::from_xyzw(w.x, w.y, w.z, 0.0) * 0.5 * dt;
        self.attitude = (self.attitude + self.attitude * dq).normalize();

        if self.position.z < 0.0 {
            self.position.z = 0.0;
            self.velocity = Vec3::ZERO;
            self.angular_velocity = Vec3::ZERO;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f32 = 0.001;

    fn airborne() -> Quad {
        let mut quad = Quad::new(QuadParams {
            linear_drag: 0.0,
            ..QuadParams::default()
        });
        quad.position.z = 100.0;
        quad
    }

    #[test]
    fn free_fall() {
        let mut quad = airborne();
        for _ in 0..1000 {
            quad.step(DT, &MotorOutputs::default());
        }
        assert!((quad.velocity.z + GRAVITY).abs() < 0.01, "{}", quad.velocity.z);
        assert!(quad.specific_force().length() < 1e-4);
    }

    #[test]
    fn hover_throttle_holds_altitude() {
        let mut quad = airborne();
        let t = quad.params.hover_throttle();
        quad.motor_speeds = [t * quad.params.max_motor_speed; 4];
        for _ in 0..2000 {
            quad.step(DT, &MotorOutputs([t; 4]));
        }
        assert!(quad.acceleration.length() < 1e-3, "{}", quad.acceleration);
        assert!((quad.position.z - 100.0).abs() < 1e-3, "{}", quad.position.z);
        assert!((quad.specific_force().z - GRAVITY).abs() < 1e-3);
    }

    #[test]
    fn symmetric_input_does_not_rotate() {
        let mut quad = airborne();
        for _ in 0..2000 {
            quad.step(DT, &MotorOutputs([0.7; 4]));
        }
        assert!(quad.angular_velocity.length() < 1e-6);
        assert!(quad.attitude.angle_between(Quat::IDENTITY) < 1e-6);
    }

    #[test]
    fn rests_on_ground_when_idle() {
        let mut quad = Quad::new(QuadParams::default());
        for _ in 0..1000 {
            quad.step(DT, &MotorOutputs::default());
        }
        assert_eq!(quad.position, Vec3::ZERO);
        assert!((quad.specific_force().z - GRAVITY).abs() < 1e-6);
    }

    #[test]
    fn front_motors_pitch_nose_up() {
        let mut quad = airborne();
        for _ in 0..100 {
            quad.step(DT, &MotorOutputs([0.5, 0.6, 0.5, 0.6]));
        }
        // Nose up in FLU is negative rotation about y.
        assert!(quad.angular_velocity.y < 0.0);
    }
}
