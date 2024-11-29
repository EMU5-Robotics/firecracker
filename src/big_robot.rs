use communication::RobotInfo;
use robot_serial::protocol::{controller::*, *};

mod brain;
mod controller;
mod drivebase;
mod imu;
mod pid;

// cartesion coordinate space

fn main() {
    let _ = communication::Logger::try_init(RobotInfo::new("big robot"), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();

    let drivebase = drivebase::Drivebase::new(
        [(5, true), (4, true), (3, true)],
        [(19, false), (18, false), (17, false)],
        MotorControl::BrakeBrake,
    );

    let mut imu = imu::Imu::new(7);
    let mut imu_pid = pid::Pid::new(0.55, 0.055, 2.2);

    let mut track_pid = false;

    loop {
        let pkt = brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();
        //

        // update imu
        imu.update(&pkt);

        log::info!("{}", imu.heading());

        // intake
        let intake_speed = controller.lx();
        pkt_to_write.set_motors[9] = MotorControl::Voltage(4.0 * intake_speed);
        if intake_speed.abs() < 0.05 {
            pkt_to_write.set_motors[9] = MotorControl::BrakeHold;
        }

        if controller.pressed(A) {
            imu_pid.set_target(imu.heading() + std::f64::consts::PI * 0.5);
            track_pid = true;
        } else if controller.pressed(B) {
            track_pid = false;
        }

        if track_pid {
            let rotate = imu_pid.poll(imu.heading());
            drivebase.write_powers(0.0, -rotate, pkt_to_write);
        } else {
            drivebase.write_powers(controller.ly(), -controller.rx(), pkt_to_write);
        }

        //
        brain.write_changes();
    }
}
