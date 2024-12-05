use crate::path::PathSegment;
use crate::ToBrain;
use robot_serial::protocol::EncoderState;
use robot_serial::protocol::MotorControl;
use robot_serial::protocol::ToRobot;
use std::time::{Duration, Instant};

#[derive(Debug, Clone)]
pub struct ShakingMotor {
    update_time: Instant,
    update_interval: Duration,
    motor: usize,
    radians_buffer: [f64; 2],
    stuck_threshold: f64,
    power_buffer: [f64; 2],
    shaking_time: Instant,
    shaking_interval: Duration,
    shaking_dir: bool,
    power: f64,
}

impl ShakingMotor {
    pub fn new(
        motor: usize,
        update_interval: Duration,
        stuck_threshold: f64,
        shaking_interval: Duration,
    ) -> Self {
        Self {
            update_time: Instant::now(),
            update_interval,
            motor,
            radians_buffer: [0.0, 0.0],
            power_buffer: [0.0, 0.0],
            stuck_threshold,
            shaking_interval,
            shaking_time: Instant::now(),
            shaking_dir: false,
            power: 0.0,
        }
    }

    pub fn update(&mut self, pkt: &ToRobot) {
        if self.update_time.elapsed() >= self.update_interval {
            let EncoderState::Radians(radians) = pkt.encoder_state[self.motor - 1] else {
                log::info!("No encoder state for smart motor {}", self.motor);
                return;
            };
            self.radians_buffer[1] = radians;
            self.radians_buffer.swap(0, 1);
            self.update_time = Instant::now();
        }
    }

    pub fn is_stuck(&self) -> bool {
        (self.radians_buffer[0] - self.radians_buffer[1]).abs() > self.stuck_threshold
            && self.power_buffer.iter().sum::<f64>().abs() > self.stuck_threshold
    }

    pub fn write_powers(&mut self, pkt: &mut ToBrain) {
        if self.shaking_time.elapsed() < self.shaking_interval {
            // the motor is shaking, don't write any power
            pkt.set_motors[self.motor - 1] =
                MotorControl::Voltage(if self.shaking_dir { -12.0 } else { 12.0 });

            self.shaking_dir = !self.shaking_dir;
        } else {
            // the motor is not shaking, write the power and detect if it's stuck
            if self.is_stuck() {
                self.shaking_time = Instant::now();
            } else {
                pkt.set_motors[self.motor - 1] = MotorControl::Voltage(self.power * 12.0);
            }
        }
    }

    pub fn set_power(&mut self, power: f64) {
        self.power = power;
    }
}

impl PathSegment for ShakingMotor {
    fn finished_transform(&self) -> bool {
        true
    }

    fn start(&mut self, _: &crate::odometry::Odom, _: &mut crate::pid::Pid, pkt: &mut ToBrain) {}

    fn follow(
        &mut self,
        odom: &crate::odometry::Odom,
        _: &mut crate::pid::Pid,
        pkt: &mut ToBrain,
    ) -> crate::path::PathOutput {
        self.update(odom.last_pkt().unwrap());
        self.write_powers(pkt);
        crate::path::PathOutput::Voltages(crate::vec::Vec2::ZERO)
    }
    fn abrupt_end(&mut self, _odom: &crate::odometry::Odom, pkt: &mut ToBrain) {
        pkt.set_motors[self.motor - 1] = MotorControl::BrakeCoast;
    }
    fn end_follow<'a>(
        &mut self,
        _: &crate::odometry::Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        None
    }
}
