use std::f64::consts::TAU;

use robot_serial::protocol::{EncoderState, MotorControl, ToBrain, ToRobot};

use crate::vec::Vec2;

pub struct Drivebase<const N: usize> {
    left: [(usize, bool); N],
    right: [(usize, bool); N],
    brakemode: MotorControl,
    side_distances: Vec2,
    radius: f64,
    radians_to_mil: f64,
}

impl<const N: usize> Drivebase<N> {
    pub fn new(
        left: [(usize, bool); N],
        right: [(usize, bool); N],
        brakemode: MotorControl,
        radius: f64,
        radians_to_mil: f64,
    ) -> Self {
        for (i, _) in left.iter().chain(right.iter()) {
            assert!((1..=21).contains(i));
        }
        Self {
            left,
            right,
            brakemode,
            side_distances: Vec2::ZERO,
            radius,
            radians_to_mil,
        }
    }
    pub fn write_voltage(&self, left: f64, right: f64, brain_pkt: &mut ToBrain) {
        let map_voltage = |power: f64, rev: bool| -> MotorControl {
            if power == 0.0 {
                return self.brakemode;
            }
            let power = (power * 12.0).clamp(-12.0, 12.0);
            if rev {
                MotorControl::Voltage(-power)
            } else {
                MotorControl::Voltage(power)
            }
        };

        for (idx, rev) in &self.left {
            brain_pkt.set_motors[*idx - 1] = map_voltage(left, *rev);
        }
        for (idx, rev) in &self.right {
            brain_pkt.set_motors[*idx - 1] = map_voltage(right, *rev);
        }
    }
    pub fn write_powers(&self, forward: f64, rotate: f64, brain_pkt: &mut ToBrain) {
        let left_side = forward - (rotate * rotate) * rotate.signum();
        let right_side = forward + (rotate * rotate) * rotate.signum();

        self.write_voltage(left_side, right_side, brain_pkt);
    }
    // see https://wiki.purduesigbots.com/software/control-algorithms/ramsete#commanding-the-robot
    // note for us we rotate counterclockwise
    pub fn write_linear_angular_vel(&self, linear: f64, angular: f64, brain_pkt: &mut ToBrain) {
        let linear_angular = linear / self.radians_to_mil;
        let left = linear_angular - angular;
        let right = linear_angular + angular;

        let map_rpm = |angular_vel: f64, rev: bool| -> MotorControl {
            if angular_vel == 0.0 {
                return self.brakemode;
            }

            // convert from rad/s to rpm
            // TODO: validate if gearing is taken care of from above?
            let target_rpm = angular_vel / TAU * 60.0;

            if rev {
                MotorControl::Velocity(-target_rpm as i32)
            } else {
                MotorControl::Velocity(target_rpm as i32)
            }
        };

        for (idx, rev) in &self.left {
            brain_pkt.set_motors[*idx - 1] = (map_rpm(left, *rev));
        }
        for (idx, rev) in &self.right {
            brain_pkt.set_motors[*idx - 1] = (map_rpm(right, *rev));
        }
        // d = ang * pi
    }
    pub fn update(&mut self, pkt: &ToRobot) -> Vec2 {
        let get_dist = |motors: &[(usize, bool); N]| -> Option<f64> {
            let mut sum = 0.0;
            for (port, rev) in motors {
                let mul = if *rev { -1.0 } else { 1.0 };
                let EncoderState::Radians(radians) = pkt.encoder_state[port - 1] else {
                    return None;
                };
                sum += radians * self.radians_to_mil * mul;
            }
            Some(sum / N as f64)
        };
        let new = Vec2::new(
            get_dist(&self.left).unwrap_or(self.side_distances[0]),
            get_dist(&self.right).unwrap_or(self.side_distances[1]),
        );
        self.side_distances = new;
        new
    }
    pub fn side_distances(&self) -> Vec2 {
        self.side_distances
    }
    pub fn radius(&self) -> f64 {
        self.radius
    }
}
