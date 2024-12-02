use crate::ramsete::Ramsete;
use crate::{odometry::Odom, pid::Pid};
use std::collections::VecDeque;
use std::f64::consts::{PI, TAU};

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
    fn transform_segments(&mut self, odom: &Odom, angle_pid: &mut Pid) {
        if self.current_segment.is_some() {
            return;
        }

        while let Some(mut new_seg) = self.segments.pop_back() {
            if new_seg.finished_transform() {
                log::info!("started new segment: {new_seg:?}");
                new_seg.start(odom, angle_pid);
                self.current_segment = Some(new_seg);
                return;
            }
            self.segments.extend(new_seg.transform(odom));
        }
    }
    pub fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2] {
        // get new segments if needed
        self.transform_segments(odom, angle_pid);

        // exit when no segments could be transformed
        let Some(seg) = self.current_segment.as_mut() else {
            return [0.0; 2];
        };

        // end segment and start next
        if let Some(new_segments) = seg.end_follow(odom) {
            if new_segments.is_empty() {
                log::info!("segment_ended: {seg:?} and added new segments: {new_segments:?}");
            } else {
                log::info!("segment_ended: {seg:?}");
            }
            self.segments.extend(new_segments);
            self.current_segment = None;
            return self.follow(odom, angle_pid);
        }

        seg.follow(odom, angle_pid)
    }
    fn abrupt_end(&mut self, odom: &Odom) {
        if let Some(seg) = self.current_segment.as_mut() {
            seg.abrupt_end(odom);
        }
    }
    pub fn ended(&self) -> bool {
        self.current_segment.is_none() && self.segments.is_empty()
    }
}

pub trait PathSegment: std::fmt::Debug {
    fn transform<'a>(self: Box<Self>, odom: &Odom) -> Vec<Box<dyn PathSegment + 'a>> {
        if self.finished_transform() {
            unreachable!("transform should never get called since finished_transform is true");
        } else {
            unimplemented!();
        }
    }
    fn finished_transform(&self) -> bool;
    fn start(&mut self, odom: &Odom, angle_pid: &mut Pid);
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2];
    fn end_follow<'a>(&mut self, odom: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>>;
    fn abrupt_end(&mut self, odom: &Odom) {}
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        panic!("This type is designed to not be clonable: {self:?}");
    }
}

impl PathSegment for Path {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid) {}
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2] {
        Path::follow(self, odom, angle_pid)
    }
    fn end_follow<'a>(&mut self, _: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.ended() {
            Some(Vec::new())
        } else {
            None
        }
    }
    fn abrupt_end(&mut self, odom: &Odom) {
        Path::abrupt_end(self, odom);
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
struct RamsetePoint {
    target: ([f64; 2], f64),
    controller: Ramsete,
}

impl PathSegment for RamsetePoint {
    fn finished_transform(&self) -> bool {
        true
    }

    fn start(&mut self, odom: &Odom, _: &mut Pid) {
        todo!()
    }

    fn follow(&mut self, odom: &Odom, _: &mut Pid) -> [f64; 2] {
        todo!()
    }

    fn end_follow<'a>(&mut self, odom: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        todo!()
    }
}

#[derive(Debug)]
struct TurnTo {
    start_heading: f64,
    target_heading: f64,
}

impl PathSegment for TurnTo {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, odom: &Odom, angle_pid: &mut Pid) {
        self.target_heading = optimise_target_heading(odom.heading(), self.target_heading);
        angle_pid.set_target(self.target_heading);
        angle_pid.reset();
    }
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2] {
        let pow = angle_pid.poll(odom.heading());
        [-pow, pow]
    }
    fn end_follow<'a>(&mut self, odom: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if (odom.heading() - self.target_heading).abs() < 2f64.to_radians() {
            log::info!(
                "Finished segment - TurnTo({}) with heading ({}).",
                self.target_heading,
                odom.heading()
            );
            return Some(vec![]);
        }
        None
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
    fn start(&mut self, _: &Odom, _: &mut Pid) {
        self.start = std::time::Instant::now();
    }
    fn follow(&mut self, _: &Odom, _: &mut Pid) -> [f64; 2] {
        [self.pow; 2]
    }
    fn end_follow<'a>(&mut self, _: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.start.elapsed() > self.dur {
            return Some(Vec::new());
        }
        None
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        Box::new(self.clone())
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
