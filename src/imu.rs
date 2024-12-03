use robot_serial::protocol::*;
pub struct Imu {
    port: usize,
    original_heading: Option<f64>,
    heading: f64,
}

impl Imu {
    pub fn new(port: usize) -> Self {
        assert!((1..=21).contains(&port));
        Self {
            port,
            original_heading: None,
            heading: 0.0,
        }
    }
    pub fn heading(&self) -> f64 {
        self.heading * std::f64::consts::PI / 180.0
    }
    pub fn heading_degrees(&self) -> f64 {
        self.heading
    }
    pub fn update(&mut self, pkt: &ToRobot) {
        let ImuState::State { z_rotation, .. } = pkt.imu_state[self.port - 1] else {
            return;
        };
        let z_rotation = -z_rotation;

        match self.original_heading {
            Some(og_heading) => {
                self.heading = z_rotation - og_heading;
            }
            None => {
                self.original_heading = Some(z_rotation);
            }
        }
    }
}
