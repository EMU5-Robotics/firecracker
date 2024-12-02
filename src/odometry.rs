use std::{
    env::args,
    mem::swap,
    time::{Duration, Instant},
};

#[derive(Debug)]
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
}
