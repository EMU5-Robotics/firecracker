use robot_serial::protocol::{self, *};

// cartesion coordinate space

pub struct Drivebase<const N: usize> {
    left: [(usize, bool); N],
    right: [(usize, bool); N],
}

impl<const N: usize> Drivebase<N> {
    pub fn new(left: [(usize, bool); N], right: [(usize, bool); N]) -> Self {
        Self { left, right }
    }
    pub fn write_powers(&self, forward: f64, rotate: f64, brain_pkt: &mut ToBrain) {
        let left_side = forward - rotate;
        let right_side = forward + rotate;

        let map_voltage = |power: f64, rev: bool| -> f64 {
            let power = 0.3 * (power * 12.0).clamp(-12.0, 12.0);
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

fn main() {
    let mut bm = robot_serial::BrainMediator::new().unwrap();
    let mut write_brain = protocol::ToBrain::default();
    let mut brain_state = protocol::ToRobot::default();

    let (mut forward, mut rotate) = (0.0, 0.0);

    let drivebase = Drivebase::new(
        [(0, true), (1, true), (2, true)],
        [(10, false), (11, false), (12, false)],
    );
    loop {
        match bm.try_read() {
            Ok(mut pkts) => {
                if let Some(pkt) = pkts.pop() {
                    if pkts.len() > 1 {
                        log::warn!("BrainMediator::try_read read {} packets", pkts.len());
                    }
                    brain_state = pkt;
                }
            }
            Err(e) => log::error!("BrainMediator::try_read returned: {e}"),
        }

        if let Some(ref controller) = brain_state.controller_state {
            forward = controller.axis[1];
            rotate = -controller.axis[2];

            if controller.buttons & controller::A == controller::A {
                write_brain.set_triports[0] = ConfigureAdiPort::DigitalHigh;
                write_brain.set_triports[1] = ConfigureAdiPort::DigitalHigh;
            } else if controller.buttons & controller::B == controller::B {
                write_brain.set_triports[0] = ConfigureAdiPort::DigitalLow;
                write_brain.set_triports[1] = ConfigureAdiPort::DigitalLow;
            }
        }

        drivebase.write_powers(forward, rotate, &mut write_brain);

        if let Err(e) = bm.try_write(&write_brain) {
            log::error!("Failed to write packet: {e}");
        }
    }
}
