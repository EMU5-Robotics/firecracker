use robot_serial::protocol::ToBrain;

use crate::{odometry::Odom, path::*, pid::Pid, vec::Vec2};

#[derive(Debug, Clone, Copy)]
pub struct Nop {}

impl PathSegment for Nop {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {}
    fn follow(&mut self, _: &Odom, _: &mut Pid, _: &mut ToBrain) -> PathOutput {
        PathOutput::Voltages(Vec2::ZERO)
    }
    fn end_follow<'a>(
        &mut self,
        _: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        None
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        Box::new(*self)
    }
}

#[derive(Debug)]
pub struct RepeatSegment {
    max_count: usize,
    count: usize,
    ref_seg: Box<dyn PathSegment>,
    current_seg: Box<dyn PathSegment>,
}

impl RepeatSegment {
    pub fn new(path: Box<dyn PathSegment>, max_count: usize) -> Self {
        Self {
            max_count,
            count: 0,
            current_seg: path.boxed_clone(),
            ref_seg: path.boxed_clone(),
        }
    }
}

impl PathSegment for RepeatSegment {
    fn finished_transform(&self) -> bool {
        true
    }
    // This assures self.ref_seg is of type "Path" (to avoid handling
    // segment start and transform code)
    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {
        let a: Box<Path> = Box::new(self.ref_seg.boxed_clone().into());
        self.ref_seg = a;
        self.current_seg = self.ref_seg.boxed_clone();
    }
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        self.current_seg.follow(odom, angle_pid, pkt)
    }
    fn end_follow<'a>(
        &mut self,
        odom: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        let ret = self.current_seg.end_follow(odom, pkt)?;

        if ret.is_empty() && self.count != self.max_count {
            self.count += 1;
            self.current_seg = self.ref_seg.boxed_clone();
            return None;
        }

        Some(ret)
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        Box::new(Self {
            max_count: self.max_count,
            count: 0,
            current_seg: self.ref_seg.boxed_clone(),
            ref_seg: self.ref_seg.boxed_clone(),
        })
    }
}

#[derive(Debug)]
pub struct WhileSegment {
    main: Path,
    secondary: Path,
    end_secondary: bool,
}

impl WhileSegment {
    pub fn new(main: Path, secondary: Path, end_secondary: bool) -> Self {
        Self {
            main,
            secondary,
            end_secondary,
        }
    }
}

impl PathSegment for WhileSegment {
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid, pkt: &mut ToBrain) {}
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        let _ = self.secondary.follow(odom, angle_pid, pkt);
        self.main.follow(odom, angle_pid, pkt)
    }
    fn end_follow<'a>(
        &mut self,
        odom: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if let Some(t) = self.main.end_follow(odom, pkt) {
            if self.end_secondary && !self.secondary.ended() {
                self.secondary.abrupt_end(odom, pkt);
            }
            return Some(t);
        }
        None
    }
    fn abrupt_end(&mut self, odom: &Odom, pkt: &mut ToBrain) {
        self.main.abrupt_end(odom, pkt);
        self.secondary.abrupt_end(odom, pkt);
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        todo!()
    }
}

#[derive(Debug)]
pub struct TimedSegment {
    seg: Box<dyn PathSegment>,
    dur: std::time::Duration,
    start: std::time::Instant,
}

impl TimedSegment {
    pub fn new(seg: Box<dyn PathSegment>, dur: std::time::Duration) -> Self {
        Self {
            seg,
            dur,
            start: std::time::Instant::now(),
        }
    }
}

impl PathSegment for TimedSegment {
    fn transform<'a>(self: Box<Self>, odom: &Odom) -> Vec<Box<dyn PathSegment + 'a>> {
        self.seg.transform(odom)
    }
    fn finished_transform(&self) -> bool {
        self.seg.finished_transform()
    }
    fn start(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) {
        self.start = std::time::Instant::now();
        self.seg.start(odom, angle_pid, pkt);
    }
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid, pkt: &mut ToBrain) -> PathOutput {
        self.seg.follow(odom, angle_pid, pkt)
    }
    fn end_follow<'a>(
        &mut self,
        odom: &Odom,
        pkt: &mut ToBrain,
    ) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if self.start.elapsed() > self.dur {
            self.seg.abrupt_end(odom, pkt);
            return Some(Vec::new());
        }
        self.seg.end_follow(odom, pkt)
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        Box::new(Self {
            seg: self.seg.as_ref().boxed_clone(),
            dur: self.dur,
            start: self.start,
        })
    }
}
