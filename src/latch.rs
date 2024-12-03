use std::{
    ops::RangeBounds,
    time::{Duration, Instant},
};

use robot_serial::protocol::{AdiPortState, ConfigureAdiPort, MotorControl, ToBrain};

use crate::path::PathSegment;

#[derive(Debug, Clone)]
pub enum Latch {
    Air {
        port: usize,
        active: bool,
        rev: bool,
    },
    Motor {
        port: usize,
        last_change: Instant,
        active: bool,
        rev: bool,
    },
}

impl Latch {
    const MOTOR_TIME: Duration = Duration::from_millis(50);
    const VOLTAGE: f64 = 3.0;
    pub fn new_air(port: usize, rev: bool) -> Self {
        assert!((1..=8).contains(&port));
        Self::Air {
            port,
            active: false,
            rev,
        }
    }
    pub fn new_motor(port: usize, rev: bool) -> Self {
        assert!((1..=21).contains(&port));
        Self::Motor {
            port,
            last_change: Instant::now(),
            active: false,
            rev,
        }
    }
    pub fn grab(&mut self) {
        match self {
            Self::Air { active, rev, .. } => {
                *active = !*rev;
            }
            Self::Motor {
                active,
                last_change,
                rev,
                ..
            } => {
                *active = !*rev;
                *last_change = Instant::now();
            }
        }
    }
    pub fn release(&mut self) {
        match self {
            Self::Air { active, rev, .. } => {
                *active = *rev;
            }
            Self::Motor {
                active,
                last_change,
                rev,
                ..
            } => {
                *active = *rev;
                *last_change = Instant::now();
            }
        }
    }
    pub fn toggle(&mut self) {
        match self {
            Self::Air { active, .. } => {
                *active = !*active;
            }
            Self::Motor {
                active,
                last_change,
                ..
            } => {
                *active = !*active;
                *last_change = Instant::now();
            }
        }
    }
    pub fn write_pkt(&self, pkt: &mut ToBrain) {
        match self {
            Self::Air { port, active, rev } => {
                pkt.set_triports[port - 1] = if active != rev {
                    ConfigureAdiPort::DigitalHigh
                } else {
                    ConfigureAdiPort::DigitalLow
                }
            }
            Self::Motor {
                port,
                last_change,
                active,
                rev,
            } => {
                if last_change.elapsed() > Self::MOTOR_TIME {
                    pkt.set_motors[port - 1] = MotorControl::BrakeBrake;
                    return;
                }
                if active != rev {
                    pkt.set_motors[port - 1] = MotorControl::Voltage(Self::VOLTAGE);
                } else {
                    pkt.set_motors[port - 1] = MotorControl::Voltage(-Self::VOLTAGE);
                }
            }
        }
    }
}

#[derive(Debug, Clone)]
pub struct LatchAction {
    latch: Latch,
    release: bool,
    wrote: bool,
}

impl LatchAction {
    pub fn new(latch: Latch, release: bool) -> Self {
        Self {
            latch,
            release,
            wrote: false,
        }
    }
}

impl PathSegment for LatchAction {
    fn finished_transform(&self) -> bool {
        true
    }

    fn start(&mut self, _: &crate::odometry::Odom, _: &mut crate::pid::Pid, pkt: &mut ToBrain) {}

    fn follow(
        &mut self,
        _: &crate::odometry::Odom,
        _: &mut crate::pid::Pid,
        pkt: &mut ToBrain,
    ) -> crate::path::PathOutput {
        if self.release {
            self.latch.release();
        } else {
            self.latch.grab()
        }
        self.latch.write_pkt(pkt);
        self.wrote = true;
        crate::path::PathOutput::Voltages(crate::vec::Vec2::ZERO)
    }

    fn end_follow<'a>(
        &mut self,
        _: &crate::odometry::Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.wrote {
            Some(Vec::new())
        } else {
            None
        }
    }
}
