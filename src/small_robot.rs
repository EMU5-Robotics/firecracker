use communication::RobotInfo;
use robot_serial::protocol::{self, *};

mod drivebase;

// cartesion coordinate space

fn main() {
    let mut bm = robot_serial::BrainMediator::new().unwrap();
    let mut cm = communication::Logger::try_init(RobotInfo::new("small robot"), true).unwrap();
    let mut write_brain = protocol::ToBrain::default();
    let mut brain_state = protocol::ToRobot::default();

    let mut intake_speed = 0.0;

    let (mut forward, mut rotate) = (0.0, 0.0);

    let drivebase = drivebase::Drivebase::new(
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
            intake_speed = controller.axis[0];
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
        write_brain.set_motors[7] = MotorControl::Voltage(12.0 * intake_speed);

        drivebase.write_powers(forward, rotate, &mut write_brain);

        if let Err(e) = bm.try_write(&write_brain) {
            log::error!("Failed to write packet: {e}");
        }
    }
}
