use robot_serial::protocol::{MotorControl, ToBrain};
pub struct Drivebase<const N: usize> {
    pub left: [(usize, bool); N],
    pub right: [(usize, bool); N],
    brakemode: MotorControl,
}

impl<const N: usize> Drivebase<N> {
    pub fn new(
        left: [(usize, bool); N],
        right: [(usize, bool); N],
        brakemode: MotorControl,
    ) -> Self {
        for (i, _) in left.iter().chain(right.iter()) {
            assert!((1..=21).contains(i));
        }
        Self {
            left,
            right,
            brakemode,
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
}
