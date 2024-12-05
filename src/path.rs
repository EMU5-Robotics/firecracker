use robot_serial::protocol::{MotorControl, ToBrain};

use crate::ramsete::Ramsete;
use crate::TimedSegment;
use crate::{odometry::Odom, pid::Pid, vec::Vec2};
use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};
use std::time::{Duration, Instant};

#[derive(Debug, Copy, Clone)]
pub enum PathOutput {
    Voltages(Vec2),
    LinearAngularVelocity(Vec2),
    SwitchToDriver,
}

#[derive(Debug)]
pub struct Path {
    // this is a stack so the last element in
    // the vector is the first that will be run
    pub segments: VecDeque<Box<dyn PathSegment>>,
    pub current_segment: Option<Box<dyn PathSegment>>,
}

impl Path {
    pub fn new(reversed_segments: Vec<Box<dyn PathSegment>>) -> Self {
        Self {
            segments: reversed_segments.into_iter().rev().collect(),
            current_segment: None,
        }
    }
    pub fn extend(&mut self, v: Box<dyn PathSegment>) {
        self.segments.push_front(v);
    }
    pub fn extend_front(&mut self, v: Box<dyn PathSegment>) {
        self.segments.push_back(v);
    }
}

#[macro_export]
macro_rules! path {
    ($($item:expr),* $(,)?) => {
            $crate::path::Path::new(vec![$(Box::new($item)),+])
        }
}

impl From<Box<dyn PathSegment>> for Path {
    fn from(seg: Box<dyn PathSegment>) -> Self {
        Self {
            segments: vec![seg].into(),
            current_segment: None,
        }
    }
}

impl Path {
    fn transform_segments(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) {
        if self.current_segment.is_some() {
            return;
        }

        while let Some(mut new_seg) = self.segments.pop_back() {
            if new_seg.finished_transform() {
                log::info!("started new segment: {new_seg:?}");
                new_seg.start(odom, angle_pid, pkt);
                self.current_segment = Some(new_seg);
                return;
            }
            self.segments.extend(new_seg.transform(odom));
        }
    }
    pub fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        // get new segments if needed
        self.transform_segments(odom, angle_pid, pkt);

        // exit when no segments could be transformed
        let Some(seg) = self.current_segment.as_mut() else {
            return PathOutput::Voltages(Vec2::ZERO);
        };

        // end segment and start next
        if let Some(new_segments) = seg.end_follow(odom, pkt) {
            if new_segments.is_empty() {
                log::info!("segment_ended: {seg:?} and added new segments: {new_segments:?}");
            } else {
                log::info!("segment_ended: {seg:?}");
            }
            self.segments.extend(new_segments);
            self.current_segment = None;
            return self.follow(odom, angle_pid, pkt);
        }

        seg.follow(odom, angle_pid, pkt)
    }
    fn abrupt_end(&mut self, odom: &Odom, pkt: &mut ToBrain) {
        if let Some(seg) = self.current_segment.as_mut() {
            seg.abrupt_end(odom, pkt);
        }
    }
    pub fn ended(&self) -> bool {
        self.current_segment.is_none() && self.segments.is_empty()
    }
}

pub trait PathSegment: std::fmt::Debug {
    fn transform<'a>(self: Box<Self>, _odom: &Odom) -> Vec<Box<dyn PathSegment + 'a>> {
        if self.finished_transform() {
            unreachable!("transform should never get called since finished_transform is true");
        } else {
            unimplemented!();
        }
    }
    fn finished_transform(&self) -> bool;
    fn start(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain);
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput;
    fn end_follow<'a>(
        &mut self,
        odom: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>>;
    fn abrupt_end(&mut self, _odom: &Odom, pkt: &mut ToBrain) {}
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        panic!("This type is designed to not be clonable: {self:?}");
    }
}

impl PathSegment for Path {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {}
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        Path::follow(self, odom, angle_pid, pkt)
    }
    fn end_follow<'a>(
        &mut self,
        _: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.ended() {
            Some(Vec::new())
        } else {
            None
        }
    }
    fn abrupt_end(&mut self, odom: &Odom, pkt: &mut ToBrain) {
        Path::abrupt_end(self, odom, pkt);
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        Box::new(Self {
            segments: self
                .segments
                .iter()
                .map(|v| v.as_ref().boxed_clone())
                .collect(),
            current_segment: self
                .current_segment
                .as_ref()
                .map(|v| v.as_ref().boxed_clone()),
        })
    }
}

#[derive(Debug)]
pub struct RamsetePoint {
    target: (Vec2, f64),
    controller: Ramsete,
}

impl RamsetePoint {
    pub fn new(target: (Vec2, f64), controller: Ramsete) -> Self {
        Self { target, controller }
    }
}

impl PathSegment for RamsetePoint {
    fn finished_transform(&self) -> bool {
        true
    }

    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {
        self.controller.set_target(self.target);
    }

    fn follow(&mut self, odom: &Odom, _: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        PathOutput::LinearAngularVelocity(self.controller.output_linear_angular(odom))
    }

    fn end_follow<'a>(
        &mut self,
        odom: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        // ignore angle since that's most likely unrecoverable
        if (odom.pos() - self.target.0).mag() < 50.0 {
            return Some(vec![]);
        }
        None
    }
}

#[derive(Debug)]
pub struct RamsetePath {
    target: VecDeque<(Vec2, f64)>,
    current_target: Option<(Vec2, f64)>,
    controller: Ramsete,
}

impl RamsetePath {
    pub fn new<T: Into<VecDeque<(Vec2, f64)>>>(target: T, mut controller: Ramsete) -> Self {
        let mut target: VecDeque<_> = target.into();
        let current_target = target.pop_front();
        if let Some(v) = current_target {
            controller.set_target(v);
        }
        Self {
            target,
            current_target,
            controller,
        }
    }
}

impl PathSegment for RamsetePath {
    fn finished_transform(&self) -> bool {
        true
    }

    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {}
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        let Some(target) = self.current_target else {
            return PathOutput::Voltages(Vec2::ZERO);
        };
        let diff = odom.pos() - target.0;

        let nor = Vec2::new(target.1.cos(), target.1.sin());

        if (odom.pos() - target.0).mag() < 80.0
            && (odom.heading() - target.1).abs() < 30f64.to_radians()
            || diff.dot(nor) > 0.0
        {
            self.current_target = self.target.pop_front();
            if let Some(target) = self.current_target {
                log::error!("new ramsete target: {target:?}");
                self.controller.set_target(target);
                return self.follow(odom, angle_pid, pkt);
            }
        }

        PathOutput::LinearAngularVelocity(self.controller.output_linear_angular(odom))
    }

    fn end_follow<'a>(
        &mut self,
        _: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.current_target.is_none() {
            return Some(Vec::new());
        }
        None
    }
}

#[derive(Debug)]
pub struct TurnTo {
    target_heading: f64,
    end_time: Option<Instant>,
}
impl TurnTo {
    pub fn new(target_heading: f64) -> Self {
        Self { target_heading, end_time: None}
    }
}

impl PathSegment for TurnTo {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) {
        self.target_heading = optimise_target_heading(odom.heading(), self.target_heading);
        angle_pid.set_target(self.target_heading);
        angle_pid.reset();
    }
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        let pow = angle_pid.poll(odom.heading());
        if (odom.heading() - self.target_heading).abs() > 2f64.to_radians() {
            self.end_time = Some(Instant::now());
        }
        PathOutput::Voltages(Vec2::new(-pow, pow))
    }
    fn end_follow<'a>(
        &mut self,
        odom: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        match self.end_time {
            Some(end_time) => {

        if end_time.elapsed() > Duration::from_millis(200) {
            log::info!(
                "Finished segment - TurnTo({}) with heading ({}).",
                self.target_heading,
                odom.heading()
            );
            return Some(vec![]);
            }
            
        None
        }

        None => None,
    }
    }
}

#[derive(Debug, Clone)]
pub struct Ram {
    pow: f64,
    dur: std::time::Duration,
    start: std::time::Instant,
}

impl Ram {
    pub fn new(pow: f64, dur: std::time::Duration) -> Self {
        Self {
            pow,
            dur,
            start: std::time::Instant::now(),
        }
    }
}

impl PathSegment for Ram {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {
        self.start = std::time::Instant::now();
    }
    fn follow(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        PathOutput::Voltages(Vec2::splat(self.pow))
    }
    fn end_follow<'a>(
        &mut self,
        _: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.start.elapsed() > self.dur {
            return Some(Vec::new());
        }
        None
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        Box::new(self.clone())
    }
}
#[derive(Debug, Clone)]
pub struct PowerMotors {
    motors: Vec<usize>,
    pow: MotorControl,
}
impl PowerMotors {
    pub fn new(motors: Vec<usize>, pow: MotorControl) -> Self {
        Self { motors, pow }
    }
}

impl PathSegment for PowerMotors {
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
        for motor_idx in &self.motors {
            pkt.set_motors[motor_idx - 1] = self.pow;
        }
        crate::path::PathOutput::Voltages(crate::vec::Vec2::ZERO)
    }
    fn abrupt_end(&mut self, _odom: &Odom, pkt: &mut ToBrain) {
        for motor_idx in &self.motors {
            pkt.set_motors[motor_idx - 1] = MotorControl::BrakeCoast;
        }
    }
    fn end_follow<'a>(
        &mut self,
        _: &crate::odometry::Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        for motor_idx in &self.motors {
            pkt.set_motors[motor_idx - 1] = MotorControl::BrakeCoast;
        }
        None
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SwitchController {}

impl PathSegment for SwitchController {
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
        crate::path::PathOutput::SwitchToDriver
    }
    fn abrupt_end(&mut self, _odom: &Odom, pkt: &mut ToBrain) {
    }
    fn end_follow<'a>(
        &mut self,
        _: &crate::odometry::Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        None
    }
    
}

fn optimise_target_heading(heading: f64, target: f64) -> f64 {
    let mut delta = target - heading;
    // map delta into [-TAU, TAU]
    delta %= TAU;
    // map delta into [0, TAU]
    if delta < 0.0 {
        delta += TAU;
    }
    // map delta into [-PI, PI]
    if delta > PI {
        delta -= TAU;
    }
    heading + delta
}
