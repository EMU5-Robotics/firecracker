use std::{f64::consts::FRAC_PI_2, f64::consts::PI, time::Duration};

use communication::RobotInfo;
use imu::Imu;
use latch::LatchAction;
use modifier_path::{TimedSegment, WhileSegment};
use path::{PathSegment, PowerMotors, Ram, TurnTo};
use robot_serial::protocol::{controller::*, *};
use vec::Vec2;

mod brain;
mod controller;
mod drivebase;
mod imu;
mod latch;
mod modifier_path;
mod odometry;
mod path;
mod pid;
mod ramsete;
mod vec;

// cartesion coordinate space

fn main() {
    let _ =
        communication::Logger::try_init(RobotInfo::new("small robot", 0.45, 0.45), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();

    let mut drivebase = drivebase::Drivebase::new(
        [(1, true), (2, true), (3, false)],
        [(8, true), (9, false), (10, false)],
        MotorControl::BrakeBrake,
        150.0,
        75.0,
    );

    let latch = latch::Latch::new_air(7, false);
    let latch_two = latch::Latch::new_air(8, false);

    let first_intake = WhileSegment::new(
        path!(Ram::new(0.5, Duration::from_millis(600))),
        path!(PowerMotors::new(vec![5, 6], MotorControl::Voltage(-12.0))),
        true,
    );
    let second_intake = WhileSegment::new(
        path!(Ram::new(0.5, Duration::from_millis(500))),
        path!(PowerMotors::new(vec![5, 6], MotorControl::Voltage(12.0))),
        false,
    );

    let raise_stage_one = LatchAction::new(latch.clone(), false);

    let latch_release = LatchAction::new(latch_two.clone(), false);
    let latch_goal = LatchAction::new(latch_two.clone(), false);

    let mut auton_path = path!(
        // ensure latch is free
        //latch_release.clone(),
        // intake rings while moving forward
        //first_intake,
        // turn left to align backwards with mobile goal
        TurnTo::new(FRAC_PI_2),
        // go backwards to mobile goal
        /*Ram::new(-0.5, Duration::from_millis(500)),
        // latch onto goal
        latch_goal,
        // score 2x ringsTimedSegment::new(
        TimedSegment::new(
            Box::new(PowerMotors::new(vec![6], MotorControl::Voltage(12.0))),
            Duration::from_millis(500),
        ),
        // turn to wall stake
        TurnTo::new(PI),
        // release goal
        latch_release,
        // intake rings
        second_intake,
        // turn around to line up with wall stake
        TurnTo::new(0.0),
        // ram into wall stack
        Ram::new(-0.5, Duration::from_millis(200)),
        // score
        TimedSegment::new(
            Box::new(PowerMotors::new(vec![6], MotorControl::Voltage(12.0))),
            Duration::from_millis(500),
        ),*/
    );
    let mut imu = Imu::new(15);
    let mut odom = odometry::Odom::new(Vec2::ZERO, 0.0, &imu, &drivebase);

    //let mut angle_pid = pid::Pid::new(0.55, 0.055, 2.2);
    //0.8, 1.2, 0.0
    //let mut angle_pid = pid::Pid::new(0.5, 3.5, 0.);

    // for now, the best arguments
    let mut angle_pid = pid::Pid::new(0.5, 0.8, 0.);
    let mut pid_test_count = 0;


    // init time is used to wait for the robot to settl
    let init_time = std::time::Instant::now();

    loop {
        let (pkt, is_updated) = brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();

        // wait for the robot to settle, imu to warm up
        if init_time.elapsed() < Duration::from_secs(2) {
            // change nothing
            brain.write_changes();
            continue;
        }
        
        imu.update(&pkt);
        drivebase.update(&pkt);
        odom.update(&imu, &drivebase);
        
        if is_updated {
            log::info!("{:.2?} {:.2?} {:.2?}", odom.heading(), imu.heading(), 
        (90.0f64.to_radians() * pid_test_count as f64 - odom.heading()).to_degrees()
        );
        }

        if controller.pressed(A) {
            pid_test_count += 1;
            angle_pid.set_target(odom.heading() + 90.0f64.to_radians());
        }

        let pow = angle_pid.poll(odom.heading());
        drivebase.write_volage(-pow, pow, pkt_to_write);
        //
        //
        //if let CompState::Auton(_) = pkt.comp_state {
        /*let out = auton_path.follow(&mut odom, &mut angle_pid, pkt_to_write);
        match out {
            path::PathOutput::Voltages(v) => drivebase.write_volage(v.x, v.y, pkt_to_write),
            path::PathOutput::LinearAngularVelocity(lr) => {
                drivebase.write_powers(lr.x, lr.y, pkt_to_write)
            }
        }*/
        /*} else {
            drivebase.write_powers(controller.ly(), -controller.rx(), pkt_to_write);
        }*/

//
        brain.write_changes();
    }
}
