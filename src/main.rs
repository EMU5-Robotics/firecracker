use robot_serial::protocol;

fn main() {
    env_logger::init();
    let mut bm = robot_serial::BrainMediator::new().unwrap();
    let mut motor_power = 0.0;

    loop {
        let _ = bm.try_write(&protocol::ToBrain::RequestControllerInfo);
        let _ = bm.try_write(&protocol::ToBrain::RequestDeviceInfo);
        if let Ok(pkts) = bm.try_read() {
            for pkt in pkts {
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
        }
    }
}
