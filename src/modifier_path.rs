use crate::{odometry::Odom, path::*, pid::Pid};

#[derive(Debug, Clone, Copy)]
pub struct Nop {}

impl PathSegment for Nop {
    fn transform<'a>(self: Box<Self>, _: &Odom) -> Vec<Box<dyn PathSegment + 'a>> {
        unreachable!("transform should never get called since finished_transform is true")
    }
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid) {}
    fn follow(&mut self, _: &Odom, _: &mut Pid) -> [f64; 2] {
        [0.0, 0.0]
    }
    fn end_follow<'a>(&mut self, _: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
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
    fn transform<'a>(self: Box<Self>, _: &Odom) -> Vec<Box<dyn PathSegment + 'a>> {
        unreachable!("transform should never get called since finished_transform is true")
    }
    fn finished_transform(&self) -> bool {
        true
    }
    // This assures self.ref_seg is of type "Path" (to avoid handling
    // segment start and transform code)
    fn start(&mut self, _: &Odom, _: &mut Pid) {
        let a: Box<Path> = Box::new(self.ref_seg.boxed_clone().into());
        self.ref_seg = a;
        self.current_seg = self.ref_seg.boxed_clone();
    }
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2] {
        self.current_seg.follow(odom, angle_pid)
    }
    fn end_follow<'a>(&mut self, odom: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        let ret = self.current_seg.end_follow(odom)?;

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
    secondary_ended: bool,
}

impl PathSegment for WhileSegment {
    fn transform<'a>(self: Box<Self>, _: &Odom) -> Vec<Box<dyn PathSegment + 'a>> {
        unreachable!("transform should never get called since finished_transform is true")
    }
    fn finished_transform(&self) -> bool {
        true
    }
    fn start(&mut self, _: &Odom, _: &mut Pid) {}
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2] {
        let _ = self.secondary.follow(odom, angle_pid);
        self.main.follow(odom, angle_pid)
    }
    fn end_follow<'a>(&mut self, odom: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>> {
        if !self.secondary_ended && self.secondary.ended() {
            self.secondary_ended = true;
            self.secondary.abrupt_end(odom);
        }
        if self.main.ended() {
            return Some(Vec::new());
        }
        None
    }
    fn abrupt_end(&mut self, odom: &Odom) {
        self.main.abrupt_end(odom);
        self.secondary.abrupt_end(odom);
    }
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        todo!()
    }
}
