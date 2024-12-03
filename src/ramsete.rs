use crate::{odometry::Odom, vec::Vec2};

// see https://wiki.purduesigbots.com/software/control-algorithms/ramsete
#[derive(Debug, Clone)]
pub struct Ramsete {
    beta: f64,
    zeta: f64,
    target: (Vec2, f64),
}

impl Ramsete {
    pub fn new(beta: f64, zeta: f64) -> Self {
        Self {
            beta,
            zeta,
            target: (Vec2::ZERO, 0.0),
        }
    }
    pub fn set_target(&mut self, target: (Vec2, f64)) {
        self.target = target;
    }
    pub fn output_linear_angular(&self, odom: &Odom) -> Vec2 {
        // 3.25" wheels
        let pos = odom.pos();
        let heading = odom.heading();

        let error_pos_local = [self.target.0[0] - pos[0], self.target.0[1] - pos[1]];
        let (s, c) = heading.sin_cos();
        let error_pos = [
            error_pos_local[0] * c + error_pos_local[1] * s,
            -error_pos_local[0] * s + error_pos_local[1] * c,
        ];
        let error_heading = self.target.1 - heading;

        let (sin_error, cos_error) = error_heading.sin_cos();
        let linear_velocity = 0.01 * error_pos[0];
        let angular_velocity = 0.01 * error_heading;
        let k = 2.0
            * self.zeta
            * (angular_velocity.powi(2) + self.beta * linear_velocity.powi(2)).sqrt();

        let linear_vel = linear_velocity * cos_error + k * error_pos[0];
        let angular_vel = angular_velocity
            + k * error_heading
            + (self.beta * linear_velocity * sin_error * error_pos[1]) / error_heading;

        Vec2::new(linear_vel, angular_vel)
    }
}
