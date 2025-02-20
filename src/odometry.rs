use crate::{drivebase::Drivebase, imu::Imu, vec::Vec2};
use robot_serial::protocol::ToRobot;
use std::time::{Duration, Instant};
pub struct Odom {
    start_pos: Vec2,
    start_heading: f64,
    pos: Vec2,
    heading: f64,
    last_update: Instant,
    last_distances: Vec2,
    velocity: f64,
    last_pkt: Option<ToRobot>,
}

impl Odom {
    const STRAIGHT_THRESHOLD: f64 = 0.5f64.to_radians();
    const UPDATE_RATE: Duration = Duration::from_millis(10);
    pub fn new<const N: usize>(
        start_pos: Vec2,
        start_heading: f64,
        imu: &Imu,
        drivebase: &Drivebase<N>,
    ) -> Self {
        let heading = imu.heading();
        Self {
            start_pos,
            start_heading,
            pos: [0.0; 2].into(),
            heading,
            last_distances: drivebase.side_distances(),
            last_update: Instant::now(),
            velocity: 0.0,
            last_pkt: None,
        }
    }
    pub fn update<const N: usize>(&mut self, imu: &Imu, drivebase: &Drivebase<N>, pkt: &ToRobot) {
        if self.last_update.elapsed() < Self::UPDATE_RATE {
            return;
        }
        self.last_pkt = Some(pkt.clone());

        let lr = drivebase.side_distances();
        let Vec2 { x: l, y: r } = lr;
        let Vec2 { x: dl, y: dr } = lr - self.last_distances;

        let theta = imu.heading();
        let dtheta = theta - self.heading;

        let rad = drivebase.radius();
        let side_diff = r - l;

        let side_threshold = Self::STRAIGHT_THRESHOLD * (2.0 * rad);
        let local_dx;
        let local_dy;
        let cond = dtheta < Self::STRAIGHT_THRESHOLD || side_diff < side_threshold;
        //let cond = true;

        if cond {
            // straight approximation
            local_dx = 0.5 * (dl + dr);
            local_dy = 0.0;
        } else {
            // curved approximation
            let (sin, cos) = dtheta.sin_cos();
            let rt = (rad * dl + rad * dr) / side_diff;
            local_dx = rt * sin;
            local_dy = rt * (1.0 - cos);
            // alternative approach from https://wiki.purduesigbots.com/software/odometry
            //local_dx = 2.0 * (dtheta * 0.5).sin() * (dr/dtheta + rad);
            //local_dy = 0.0
        };

        // use average theta not end theta for line segment
        let average_theta = self.start_heading + theta + dtheta * 0.5;

        let (sin, cos) = average_theta.sin_cos();

        let global_dx = cos * local_dx - sin * local_dy;
        let global_dy = sin * local_dx + cos * local_dy;

        self.pos[0] += global_dx;
        self.pos[1] += global_dy;
        self.velocity = (global_dy.powi(2) + global_dx.powi(2)).sqrt()
            / self.last_update.elapsed().as_secs_f64();

        self.last_distances = lr;
        self.heading = theta;
        self.last_update = Instant::now();
    }
    pub fn pos(&self) -> Vec2 {
        self.start_pos + self.pos
    }
    pub fn heading(&self) -> f64 {
        self.start_heading + self.heading
    }
    pub fn velocity(&self) -> f64 {
        self.velocity
    }
    pub fn last_pkt(&self) -> Option<&ToRobot> {
        self.last_pkt.as_ref()
    }
}
/*#[derive(Debug)]
pub struct Odometry {
    x: f64,
    y: f64,
    dir_change: [f64; 2],
    distance_change: [[f64; 2]; 2],
    interval: Duration,   // set time gap
    last_update: Instant, // record the time.
}

impl Default for Odometry {
    fn default() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            distance_change: [[0.0, 0.0], [0.0, 0.0]],
            dir_change: [0.0, 0.0],
            interval: Duration::from_millis(1),
            last_update: Instant::now(),
        }
    }
}

impl Odometry {
    pub fn new(interval_millis: u64) -> Self {
        Self {
            interval: Duration::from_millis(interval_millis),
            ..Default::default()
        }
    }

    fn pure_noimu_str_update_xy(&mut self) {
        // error alg
        let rlrr: f64 = 254.0;
        let left_dis_delta = self.distance_change[0][0] - self.distance_change[1][0];
        let right_dis_delta = self.distance_change[0][1] - self.distance_change[1][1];
        let mut dir_delta = (right_dis_delta - left_dis_delta) / (2.0 * rlrr);

        let avg_dis_delta: f64 = (right_dis_delta + left_dis_delta) / 2.0;
        self.x += dir_delta.cos() * avg_dis_delta;
        self.y += dir_delta.sin() * avg_dis_delta;
    }

    fn pure_imu_str_update_xy(&mut self) {
        let left_dis_delta = self.distance_change[0][0] - self.distance_change[1][0];
        let right_dis_delta = self.distance_change[0][1] - self.distance_change[1][1];

        let avg_dis_delta: f64 = (right_dis_delta + left_dis_delta) / 2.0;
        self.x += self.dir_change[0].cos() * avg_dis_delta;
        self.y += self.dir_change[0].sin() * avg_dis_delta;
    }

    fn imu_mix_update_xy(&mut self) {
        let rlrr: f64 = 254.0;
        let left_dis_delta = self.distance_change[0][0] - self.distance_change[1][0];
        let right_dis_delta = self.distance_change[0][1] - self.distance_change[1][1];
        let dir_delta = (right_dis_delta - left_dis_delta) / (2.0 * rlrr);

        if (dir_delta * (2.0 * rlrr)) < 1.0 {
            //if (right_dis_delta - left_dis_delta).abs() < 1.0 {
            let avg_dis_delta: f64 = (right_dis_delta + left_dis_delta) / 2.0;
            self.x += self.dir_change[0].cos() * avg_dis_delta;
            self.y += self.dir_change[0].sin() * avg_dis_delta;
        } else {
            let rt: f64 =
                ((left_dis_delta * rlrr) + (right_dis_delta * rlrr)) / (dir_delta * (2.0 * rlrr));
            // / (right_dis_delta - left_dis_delta);
            self.x += rt * dir_delta.sin();
            self.y += rt * (1.0 - dir_delta.cos());
        }
    }

    /*
     * The parameters dir: f64 and dis: f64 represent the direction in radians and the distance in millimeters, respectively. Please note that both parameters are cumulative values, not incremental ones; they are accumulated values from the start of the robot's operation.
     * */
    pub fn update(&mut self, dir: f64, dis: [f64; 2]) {
        if Instant::now().duration_since(self.last_update) >= self.interval {
            self.last_update = Instant::now();
            self.distance_change[1] = dis;
            self.distance_change.swap(0, 1);
            self.dir_change[1] = dir;
            self.dir_change.swap(0, 1);

            self.pure_imu_str_update_xy();
            communication::plot!("dim", "x", self.x);
            communication::plot!("dim", "y", self.y);
        }
    }

    pub fn get_xy(&self) -> [f64; 2] {
        return [self.x, self.y];
    }
}*/
