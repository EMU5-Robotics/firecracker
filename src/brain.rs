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

        let first = loop {
            if let Ok(pkt) = mediator.try_read() {
                break pkt.to_owned();
            }
        };

        let second = loop {
            if let Ok(pkt) = mediator.try_read() {
                break pkt.to_owned();
            }
        };

        let packet_buffer = [first, second];
        let controller = packet_buffer.clone().into();
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
    pub fn update_state(&mut self, controller: &mut Controller) -> (ToRobot, bool) {
        controller.update_no_change();
        let mut matched_ok = false;
        match self.mediator.try_read() {
            Ok(pkt) => {
                self.failed_read = false;
                self.packet_buffer[1] = pkt.to_owned();
                self.packet_buffer.swap(0, 1);
                *controller = self.packet_buffer.clone().into();
                matched_ok = true;
            }
            Err(robot_serial::Error::NoPacketRead) => {}
            Err(e) => {
                if !self.failed_read {
                    log::warn!("First failed read due to: {e}");
                    self.failed_read = true;
                }
            }
        }
        (self.packet_buffer[0].clone(), matched_ok)
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
