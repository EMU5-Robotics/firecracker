use communication::RobotInfo;
use robot_serial::protocol::{controller::*, *};

mod brain;
mod controller;
mod drivebase;

// cartesion coordinate space

fn main() {
    let _ =
        communication::Logger::try_init(RobotInfo::new("small robot", 0.45, 0.45), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();

    let drivebase = drivebase::Drivebase::new(
        [(1, true), (2, true), (3, true)],
        [(11, false), (12, false), (13, false)],
        MotorControl::BrakeBrake,
        150.0,
    );

    loop {
        brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();
        //

        let intake_speed = controller.lx();

        pkt_to_write.set_motors[7] = MotorControl::Voltage(12.0 * intake_speed);
        if controller.pressed(A) {
            log::info!("brake set to BrakeBrake");
            pkt_to_write.set_triports[0] = ConfigureAdiPort::DigitalHigh;
            pkt_to_write.set_triports[1] = ConfigureAdiPort::DigitalHigh;
            pkt_to_write.set_motors[18] = MotorControl::BrakeBrake;
        } else if controller.pressed(B) {
            log::info!("brake set to BrakeBrake");
            pkt_to_write.set_triports[0] = ConfigureAdiPort::DigitalLow;
            pkt_to_write.set_triports[1] = ConfigureAdiPort::DigitalLow;
        }

        drivebase.write_powers(controller.ly(), -controller.rx(), pkt_to_write);

        //
        brain.write_changes();
    }
}
