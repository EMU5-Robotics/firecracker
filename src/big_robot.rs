use communication::RobotInfo;
use robot_serial::protocol::{controller::*, *};

mod brain;
mod controller;
mod drivebase;

// cartesion coordinate space

fn main() {
    let _ = communication::Logger::try_init(RobotInfo::new("big robot"), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();

    let drivebase = drivebase::Drivebase::new(
        [(6, true), (7, true), (9, true)],
        [(13, false), (14, false), (15, false)],
    );

    loop {
        brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();
        //

        let intake_speed = controller.lx();

        pkt_to_write.set_motors[18] = MotorControl::Voltage(4.0 * intake_speed);
        if controller.pressed(A) {
            log::info!("brake set to BrakeBrake");
            pkt_to_write.set_motors[18] = MotorControl::BrakeBrake;
        } else if controller.pressed(B) {
            log::info!("brake set to BrakeBrake");
            pkt_to_write.set_motors[18] = MotorControl::BrakeHold;
        }

        drivebase.write_powers(controller.ly(), -controller.rx(), pkt_to_write);

        //
        brain.write_changes();
    }
}
