use communication::RobotInfo;
use robot_serial::protocol::{controller::*, *};

mod brain;
mod controller;
mod drivebase;

// cartesion coordinate space

fn main() {
    let _ = communication::Logger::try_init(RobotInfo::new("small robot"), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();
    let mut was_powered = false;

    loop {
        let pkt = brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();
        //
        //log::info!("{:?}", pkt.imu_state);

        let motor_speed = controller.lx();
        if motor_speed != 0.0 {
            was_powered = true;
            pkt_to_write.set_motors[18] = MotorControl::Voltage(12.0 * motor_speed);
        } else if was_powered {
            was_powered = false;
            pkt_to_write.set_motors[18] = MotorControl::BrakeBrake;
        }
        if controller.pressed(A) {
            log::info!("brake set to BrakeHold");
            pkt_to_write.set_motors[18] = MotorControl::BrakeHold;
        } else if controller.pressed(B) {
            log::info!("brake set to BrakeBrake");
            pkt_to_write.set_motors[18] = MotorControl::BrakeBrake;
        } else if controller.pressed(X) {
            log::info!("brake set to BrakeCoast");
            pkt_to_write.set_motors[18] = MotorControl::BrakeCoast;
        }

        //
        brain.write_changes();
    }
}
