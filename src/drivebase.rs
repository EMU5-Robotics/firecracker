use robot_serial::protocol::{EncoderState, MotorControl, ToBrain, ToRobot};
pub struct Drivebase<const N: usize> {
    left: [(usize, bool); N],
    right: [(usize, bool); N],
    brakemode: MotorControl,
    side_distances: [f64; 2],
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
            side_distances: [0.0; 2],
            radius,
            radians_to_mil,
        }
    }
    pub fn write_powers(&self, forward: f64, rotate: f64, brain_pkt: &mut ToBrain) {
        let left_side = forward - (rotate * rotate) * rotate.signum();
        let right_side = forward + (rotate * rotate) * rotate.signum();

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
            brain_pkt.set_motors[*idx - 1] = (map_voltage(left_side, *rev));
        }
        for (idx, rev) in &self.right {
            brain_pkt.set_motors[*idx - 1] = (map_voltage(right_side, *rev));
        }
    }
    pub fn update(&mut self, pkt: &ToRobot) -> [f64; 2] {
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
        let new = [
            get_dist(&self.left).unwrap_or(self.side_distances[0]),
            get_dist(&self.right).unwrap_or(self.side_distances[1]),
        ];
        self.side_distances = new;
        new
    }
    pub fn side_distances(&self) -> [f64; 2] {
        self.side_distances
    }
    pub fn radius(&self) -> f64 {
        self.radius
    }
}
