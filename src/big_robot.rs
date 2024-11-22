use communication::RobotInfo;
use robot_serial::protocol::{self, *};

mod drivebase;

// cartesion coordinate space

fn main() {
    let mut bm = robot_serial::BrainMediator::new().unwrap();
    let _ = communication::Logger::try_init(RobotInfo::new("big robot"), true).unwrap();
    let mut write_brain = protocol::ToBrain::default();
    let mut brain_state = protocol::ToRobot::default();

    let mut intake_speed = 0.0;

    let (mut forward, mut rotate) = (0.0, 0.0);

    let drivebase = drivebase::Drivebase::new(
        [(10, true), (13, true), (14, true)],
        [(7, false), (8, false), (9, false)],
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
            intake_speed = controller.axis[0];
            forward = controller.axis[1];
            rotate = -controller.axis[2];
        }
        write_brain.set_motors[18] = MotorControl::Voltage(12.0 * intake_speed);
        drivebase.write_powers(forward, rotate, &mut write_brain);

        if let Err(e) = bm.try_write(&write_brain) {
            log::error!("Failed to write packet: {e}");
        }
    }
}
