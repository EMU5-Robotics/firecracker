use std::{
    f64::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use communication::RobotInfo;
use imu::Imu;
use latch::LatchAction;
use modifier_path::{Nop, TimedSegment, WhileSegment};
use path::{PowerMotors, Ram, SwitchController, TurnTo};
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
mod shaking_motor;
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

    let mut front_latch = latch::Latch::new_air(8, false);
    let mut back_latch = latch::Latch::new_air(7, false);
    let front_latch_release = LatchAction::new(front_latch.clone(), true);
    let front_latch_attach = LatchAction::new(front_latch.clone(), false);
    let back_latch_release = LatchAction::new(back_latch.clone(), true);
    let back_latch_attach = LatchAction::new(back_latch.clone(), false);

    let wait_n = |v: Duration| TimedSegment::new(Box::new(Nop {}), v);

    let init_front_latch = path!(
        front_latch_attach.clone(),
        wait_n(Duration::from_secs(1)),
        front_latch_release. clone(),
        wait_n(Duration::from_secs(1)),
    );

    let get_first_ring = path!(Ram::new(0.5, Duration::from_millis(560)));

    let score_two_rings = path!(
        // turn left to align backwards with mobile goal
        //TurnTo::new(FRAC_PI_2),
        // turn 90 -> 97
        path::PowerSide::new(96.0, false),
        back_latch_release.clone(),
        // go backwards to mobile goal
        Ram::new(-0.2, Duration::from_millis(7000)),
        // latch onto goal
        back_latch_attach.clone(),
        // score 2x ringsTimedSegment::new(
        TimedSegment::new(
            Box::new(PowerMotors::new(vec![6], MotorControl::Voltage(-12.0))),
            Duration::from_millis(3000),
        ),
    );

    let stage_one = WhileSegment::new(
        path!(get_first_ring, score_two_rings),
        path!(PowerMotors::new(vec![5], MotorControl::Voltage(-5.0))),
        true,
    );

    let turn_to_last_ring = path!(
        TurnTo::new(0.75 * PI),
        // release the moveble goal
        back_latch_release.clone(),
    );

    let get_last_ring = path!(Ram::new(0.5, Duration::from_millis(500)));

    let stage_two = WhileSegment::new(
        path!(
            turn_to_last_ring,
            get_last_ring,
            wait_n(Duration::from_secs(1))
        ),
        path!(PowerMotors::new(vec![5], MotorControl::Voltage(-12.0))),
        true,
    );

    let turn_to_wall_stake = path!(TurnTo::new(5.0f64.to_radians()));
    let ram_into_wall_stake = path!(Ram::new(-0.1, Duration::from_millis(2000)));
    let score_last_ring = path!(TimedSegment::new(
        Box::new(PowerMotors::new(vec![5, 6], MotorControl::Voltage(-12.0))),
        Duration::from_millis(3000),
    ));
    let turn_to_ladder = path!(TurnTo::new(0.0));
    let ram_into_ladder = path!(Ram::new(0.2, Duration::from_millis(1500)));

    let option_b = path!(
        turn_to_wall_stake,
        ram_into_wall_stake,
        score_last_ring,
        turn_to_ladder,
        ram_into_ladder,
    );

    let turn_to_new_point = path!(TurnTo::new(-135.0f64.to_radians()));
    let ram_to_new_point = path!(Ram::new(-0.1, Duration::from_millis(1500)));
    let turn_to_new_point_two = path!(TurnTo::new(-170.0f64.to_radians()));
    let score_last_ring = path!(
        back_latch_release.clone(),
        Ram::new(-0.2, Duration::from_millis(1500)),
        back_latch_attach.clone(),
        TimedSegment::new(
            Box::new(PowerMotors::new(vec![6], MotorControl::Voltage(-12.0))),
            Duration::from_millis(3000),
        )
    );
    let turn_to_ladder = path!(
        TurnTo::new(90.0f64.to_radians()),
        back_latch_release.clone()
    );
    let ram_to_ladder = path!(Ram::new(0.2, Duration::from_millis(1500)));

    let option_c = path!(
        turn_to_new_point,
        ram_to_new_point,
        turn_to_new_point_two,
        WhileSegment::new(
            score_last_ring,
            path!(PowerMotors::new(vec![5], MotorControl::Voltage(-12.0))),
            true,
        ),
        turn_to_ladder,
        ram_to_ladder,
    );

    let mut auton_path = path!( 
        // init the front latch
        
        /* 
        init_front_latch,
        WhileSegment::new(
            /*path!(
            Ram::new(0.5, Duration::from_millis(560)),*/
            //TurnTo::new(PI),
            path!(path::PowerSide { start: std::time::Instant::now() }),
            // turn left to align backwards with mobile goal
            /*back_latch_release.clone(),
            // go backwards to mobile goal
            Ram::new(-0.2, Duration::from_millis(3000)),
            // latch onto goal
            back_latch_attach.clone(),
            // score 2x ringsTimedSegment::new(
            PowerMotors::new(vec![6], MotorControl::Voltage(-12.0)),
        ),*/
        path!(PowerMotors::new(vec![5], MotorControl::Voltage(-12.0))), true),
    )   

        */

        
        // init the front latch
        init_front_latch,
        // auton stage one
        // get first ring and score two rings
        stage_one,
        //SwitchController {},
        // auton stage two
        // turn to last ring and get last ring
        //stage_two,
        // option b
        // turn to wall stake, ram into wall stake
        // score ring, turn to ladder, ram into ladder
        //option_b,
        // option c
        // turn to new point, ram into new point, turn to new point 2
        // score last ring, turn to ladder, ram into ladder
        //option_c,
    );
    let mut imu = Imu::new(15);
    let mut odom = odometry::Odom::new(Vec2::ZERO, 0.0, &imu, &drivebase);

    // for now, the best arguments
    let mut angle_pid = pid::Pid::new(0.5, 0.8, 0.);

    // init time is used to wait for the robot to settl
    let init_time = std::time::Instant::now();
    let mut finished = false;
    let mut reversed = false;

    loop {
        let (pkt, _is_updated) = brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();

        // wait for the robot to settle, imu to warm up
        if init_time.elapsed() < Duration::from_secs(2) {
            // change nothing
            brain.write_changes();
            continue;
        }


        if _is_updated {
            log::info!("imu : {:?}", pkt.imu_state);
        }

        imu.update(&pkt);
        drivebase.update(&pkt);
        odom.update(&imu, &drivebase, &pkt);

        if let CompState::Auton(_) = pkt.comp_state {
            if !finished {
                let out = auton_path.follow(&mut odom, &mut angle_pid, pkt_to_write);
                match out {
                    path::PathOutput::Voltages(v) => {
                        drivebase.write_voltage(v.x, v.y, pkt_to_write)
                    }
                    path::PathOutput::LinearAngularVelocity(lr) => {
                        drivebase.write_powers(lr.x, lr.y, pkt_to_write)
                    }
                    path::PathOutput::SwitchToDriver => finished = true,
                }
            }
        }

        if pkt.comp_state == CompState::Driver {
            if controller.pressed(DOWN) {
                reversed = !reversed;
            }

            let mut forward = controller.ly();
            let turn = controller.rx();
            if reversed {
                forward = -forward;
            }
            drivebase.write_voltage(forward + turn.powi(3), forward - turn.powi(3), pkt_to_write);

            match (
                controller.held(LEFT_TRIGGER_1),
                controller.held(LEFT_TRIGGER_2),
            ) {
                (true, false) => pkt_to_write.set_motors[4] = MotorControl::Voltage(12.0),
                (false, true) => pkt_to_write.set_motors[4] = MotorControl::Voltage(-12.0),
                _ => pkt_to_write.set_motors[4] = MotorControl::BrakeBrake,
            }

            match (
                controller.held(RIGHT_TRIGGER_1),
                controller.held(RIGHT_TRIGGER_2),
            ) {
                (true, false) => pkt_to_write.set_motors[5] = MotorControl::Voltage(12.0),
                (false, true) => pkt_to_write.set_motors[5] = MotorControl::Voltage(-12.0),
                _ => pkt_to_write.set_motors[5] = MotorControl::BrakeCoast,
            }

            if controller.pressed(B) {
                front_latch.toggle();
            }

            if controller.pressed(A) {
                back_latch.toggle();
            }
            front_latch.write_pkt(pkt_to_write);
            back_latch.write_pkt(pkt_to_write);
        }

        //
        brain.write_changes();
    }
}
