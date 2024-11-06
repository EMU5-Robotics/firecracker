use robot_serial::protocol;

fn main() {
    env_logger::init();
    let mut bm = robot_serial::BrainMediator::new().unwrap();
    let mut motor_power = 0.0;
    /*    let _ = bm
        .try_write(&protocol::ToBrain::RequestControllerInfo)
        .unwrap();
    let _ = bm.try_write(&protocol::ToBrain::RequestDeviceInfo).unwrap();*/
    let mut write_brain = protocol::ToBrain::default();
    let mut brain_state = protocol::ToRobot::default();
    loop {
        if let Ok(pkts) = bm.try_read() {
            for pkt in pkts {
                log::info!("{pkt:?}");
                brain_state = pkt;
            }
        } else {
            log::warn!("AAAAA");
        }

        if let Some(ref controller) = brain_state.controller_state {
            motor_power = controller.axis[0];
        }

        if let Some(ref device_list) = brain_state.device_list {
            for (i, device) in device_list.smart_ports.iter().enumerate() {
                if protocol::PortState::Motor == *device {
                    write_brain.set_motors[i] = protocol::MotorControl::Voltage(
                        12.0 * (i as f64 + 1.0) / 21.0 * motor_power,
                    );
                }
            }
        }

        bm.try_write(&write_brain).unwrap();
    }

    /*loop {
        if let Ok(pkts) = bm.try_read() {
            for pkt in pkts {
                println!("AAAAA");
                match pkt {
                    protocol::ToRobot::ControllerState(controller_state) => {
                        motor_power = controller_state.axis[0];
                    }
                    protocol::ToRobot::DevicesList(devices_list) => {
                        let mut arr = [protocol::MotorControl::default(); 21];
                        for (i, device) in devices_list.smart_ports.iter().enumerate() {
                            if protocol::PortState::Motor == *device {
                                arr[i] = protocol::MotorControl::Voltage(
                                    12.0 * (i as f64 + 1.0) / 21.0 * motor_power,
                                );
                            }
                        }
                        let _ = bm.try_write(&protocol::ToBrain::SetMotors(arr));
                    }
                    _ => {}
                }
            }
        } else {
            println!("FASFASFASF");
        }
    }*/
}
