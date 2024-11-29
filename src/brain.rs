use robot_serial::{protocol::*, BrainMediator};

use crate::controller::Controller;

pub struct Brain {
    mediator: BrainMediator,
    packet_buffer: [ToRobot; 2],
    last_update: std::time::Instant,
    failed_read: bool,
    to_brain: ToBrain,
}

impl Brain {
    pub fn init() -> (Self, Controller) {
        let mut failed = false;
        let mut mediator = loop {
            if let Ok(m) = BrainMediator::new() {
                break m;
            }
            if !failed {
                log::warn!("Failed to connect to the brain retrying!");
                failed = true;
            }
        };
        log::info!("Connected to the brain");

        // prompt a response from the brain
        while let Err(_) = mediator.try_write(&ToBrain::default()) {}

        let first = loop {
            if let Ok(pkt) = mediator.try_read() {
                break pkt.to_owned();
            }
        };
        while let Err(_) = mediator.try_write(&ToBrain::default()) {}

        let second = loop {
            if let Ok(pkt) = mediator.try_read() {
                break pkt.to_owned();
            }
        };

        let packet_buffer = [first, second];
        let controller = packet_buffer.clone().into();
        while let Err(_) = mediator.try_write(&ToBrain::default()) {}
        (
            Self {
                mediator,
                packet_buffer,
                last_update: std::time::Instant::now(),
                failed_read: false,
                to_brain: ToBrain::default(),
            },
            controller,
        )
    }
    pub fn update_state(&mut self, controller: &mut Controller) -> &ToRobot {
        controller.update_no_change();
        match self.mediator.try_read() {
            Ok(pkt) => {
                self.failed_read = false;
                self.packet_buffer[1] = pkt.to_owned();
                self.packet_buffer.swap(0, 1);
                *controller = self.packet_buffer.clone().into();
            }
            Err(robot_serial::Error::NoPacketRead) => {}
            Err(e) => {
                if !self.failed_read {
                    log::warn!("First failed read due to: {e}");
                    self.failed_read = true;
                }
            }
            _ => {}
        }
        &self.packet_buffer[1]
    }
    pub fn get_brain_pkt(&mut self) -> &mut ToBrain {
        &mut self.to_brain
    }
    pub fn write_changes(&mut self) {
        if let Err(e) = self.mediator.try_write(&self.to_brain) {
            log::error!("Failed to write packet: {e}");
        }
    }
}
