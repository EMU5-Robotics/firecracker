use crate::odometry::Odom;

// see https://wiki.purduesigbots.com/software/control-algorithms/ramsete
#[derive(Debug, Clone)]
pub struct Ramsete {
    beta: f64,
    zeta: f64,
    linear_velocity: f64,
    angular_velocity: f64,
    k: f64,
    target: ([f64; 2], f64),
}

impl Ramsete {
    pub fn new(beta: f64, zeta: f64, linear_velocity: f64, angular_velocity: f64) -> Self {
        let k = 2.0 * zeta * (angular_velocity.powi(2) + beta * linear_velocity.powi(2)).sqrt();
        Self {
            beta,
            zeta,
            linear_velocity,
            angular_velocity,
            k,
            target: ([0.0; 2], 0.0),
        }
    }
    pub fn set_target(&mut self, target: ([f64; 2], f64)) {
        self.target = target;
    }
    pub fn output_linear_angular(&self, odom: &Odom) -> [f64; 2] {
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

        let linear_vel = self.linear_velocity * cos_error + self.k * error_pos[0];
        let angular_vel = self.angular_velocity
            + self.k * error_heading
            + (self.beta * self.linear_velocity * sin_error * error_pos[1]) / error_heading;

        [linear_vel, angular_vel]
    }
}
