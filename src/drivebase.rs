use robot_serial::protocol::{MotorControl, ToBrain};
pub struct Drivebase<const N: usize> {
    left: [(usize, bool); N],
    right: [(usize, bool); N],
}

impl<const N: usize> Drivebase<N> {
    pub fn new(left: [(usize, bool); N], right: [(usize, bool); N]) -> Self {
        Self { left, right }
    }
    pub fn write_powers(&self, forward: f64, rotate: f64, brain_pkt: &mut ToBrain) {
        let left_side = forward - (rotate * rotate) * rotate.signum();
        let right_side = forward + (rotate * rotate) * rotate.signum();

        let map_voltage = |power: f64, rev: bool| -> f64 {
            let power = 0.5 * (power * 12.0).clamp(-12.0, 12.0);
            if rev {
                -power
            } else {
                power
            }
        };

        for (idx, rev) in &self.left {
            brain_pkt.set_motors[*idx] = MotorControl::Voltage(map_voltage(left_side, *rev));
        }
        for (idx, rev) in &self.right {
            brain_pkt.set_motors[*idx] = MotorControl::Voltage(map_voltage(right_side, *rev));
        }
    }
}
