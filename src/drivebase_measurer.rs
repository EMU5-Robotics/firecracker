use crate::drivebase::{self, Drivebase};
use robot_serial::protocol::{controller::*, *};

#[derive(Debug, Default)]
pub struct DriveBaseMeasurer {
    mapping_ratio: f64,  // from radius to mm
    distance: [f64; 2], // [left, right]
}

impl DriveBaseMeasurer {
    pub fn new(mapping_ratio: f64) -> Self {
        Self {
            mapping_ratio: mapping_ratio,
            ..Default::default()
        }
    }

    pub fn update<const N: usize>(&mut self, pkt: &ToRobot, drivetrain: &Drivebase<N>) {
        //log::info!("{:?}", pkt.encoder_state);
        let left = drivetrain
            .left
            .iter()
            .map(
                |&(port, direction)| 
                    match pkt.encoder_state[port-1] {
                        EncoderState::Radians(v) => v,
                        _ => {log::error!("ToRobot Packet Motor Port Not Match !"); 0.0}
                    } *
                     match direction {
                        false => -1.0,
                        true => 1.0,
                    } * self.mapping_ratio
                
            )
            .sum::<f64>()
            / N as f64;

        let right = drivetrain
            .right
            .iter()
            .map(
                |&(port, direction)| 
                    match pkt.encoder_state[port-1] {
                        EncoderState::Radians(v) => v,
                        _ => {log::error!("ToRobot Packet Motor Port Not Match !"); 0.0}
                    } 
                    * match direction {
                        false => -1.0,
                        true => 1.0,
                    }
                    * self.mapping_ratio
                )
            .sum::<f64>()
            / N as f64;

        self.distance = [left, right];
    }

    pub fn get_avg_distance(&self) -> f64{
        self.distance.iter().sum::<f64>() / 2.0 // 2 is left side and right side  
    }

     pub fn get_distance(&self) -> [f64;2]{
        self.distance
    }
}
