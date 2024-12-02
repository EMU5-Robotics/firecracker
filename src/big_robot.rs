use std::{f64, time::Duration};

use communication::RobotInfo;
use modifier_path::*;
use path::*;
use ramsete::Ramsete;
use robot_serial::protocol::{controller::*, *};

mod brain;
mod controller;
mod drivebase;
mod imu;
mod modifier_path;
mod odometry;
mod path;
mod pid;
mod ramsete;
mod vec;

use vec::Vec2;

// cartesion coordinate space
//
// rl and rr = 254 mm
//

fn main() {
    let mut mediator =
        communication::Logger::try_init(RobotInfo::new("big robot", 0.705, 0.45), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();

    let mut drivebase = drivebase::Drivebase::new(
        [(19, true), (14, true), (17, true)],
        [(5, false), (4, false), (3, false)],
        MotorControl::BrakeBrake,
        254.0,
        75.0,
    );

    let mut imu = imu::Imu::new(7);
    let mut imu_pid = pid::Pid::new(0.55, 0.055, 2.2);

    let mut track_pid = false;
    let ramsete = Ramsete::new(1.0, 1.0, 1.0, 1.0);

    // example path
    let _path = path!(
        Nop {},
        TimedSegment::new(Box::new(Nop {}), Duration::from_millis(200)),
        RamsetePoint::new(
            (Vec2::new(-350.0, -350.0), std::f64::consts::FRAC_PI_4),
            ramsete
        ),
    );

    //let mut drivebase_measurer = drivebase_measurer::DriveBaseMeasurer::new(75.0);
    //let mut odometry = odometry::Odometry::new(5);
    let mut odom = odometry::Odom::new(
        Vec2::new(-400.0, -400.0),
        90.0f64.to_radians(),
        &imu,
        &drivebase,
    );

    let mut latch_close: i8 = 0;

    loop {
        let (pkt, just_updated) = brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();

        // update imu, odometry stuff
        imu.update(&pkt);
        drivebase.update(&pkt);
        odom.update(&imu, &drivebase);
        //drivebase_measurer.update(&pkt, &drivebase);
        //odometry.update(imu.heading(), drivebase_measurer.get_distance());

        if just_updated {
            //log::info!("{:?}", pkt.encoder_state);
            log::info!("imu :{:?}", imu.heading());
            //log::info!("distance :{:?}", drivebase_measurer.get_avg_distance());
            //log::info!("x y :{:?}", odometry.get_xy());
            let _ = mediator.send_event(communication::packets::FromMain::Odometry(
                odom.pos().into(),
                odom.heading(),
            ));
            log::info!("latch :{:?}", latch_close);
        }

        //println!("{}", imu.heading());

        // intake
        if controller.pressed(RIGHT_TRIGGER_2) {
            latch_close = 1;
        }
        if controller.pressed(RIGHT_TRIGGER_1) {
            latch_close = 2;
        }
        if controller.pressed(X) {
            latch_close = 0;
        }

        if latch_close == 2 {
            pkt_to_write.set_motors[9] = MotorControl::Voltage(-4.0);
        } else if latch_close == 1 {
            pkt_to_write.set_motors[9] = MotorControl::Voltage(4.0);
        } else {
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
