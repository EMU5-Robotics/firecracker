use crate::{odometry::Odom, pid::Pid};

pub trait PathSegment: std::fmt::Debug {
    fn transform<'a>(self: Box<Self>, odom: &Odom) -> Vec<Box<dyn PathSegment + 'a>>;
    fn finished_transform(&self) -> bool;
    fn start(&mut self, odom: &Odom, angle_pid: &mut Pid);
    fn follow(&mut self, odom: &Odom, angle_pid: &mut Pid) -> [f64; 2];
    fn end_follow<'a>(&mut self, odom: &Odom) -> Option<Vec<Box<dyn PathSegment + 'a>>>;
    fn abrupt_end(&mut self, odom: &Odom) {}
    fn boxed_clone<'a>(&self) -> Box<dyn PathSegment + 'a> {
        panic!("This type is designed to not be clonable: {self:?}");
    }
}
