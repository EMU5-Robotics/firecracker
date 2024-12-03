use std::{f64, time::Duration};

use communication::RobotInfo;
use modifier_path::*;
use path::*;
use ramsete::Ramsete;
use robot_serial::protocol::{controller::*, *};

mod brain;
mod controller;
mod drivebase;
mod imu;
mod latch;
mod modifier_path;
mod odometry;
mod path;
mod pid;
mod ramsete;
mod vec;

use vec::Vec2;

// cartesion coordinate space
//
// rl and rr = 254 mm
//

fn main() {
    let mut mediator =
        communication::Logger::try_init(RobotInfo::new("big robot", 0.705, 0.45), true).unwrap();
    let (mut brain, mut controller) = brain::Brain::init();

    let mut drivebase = drivebase::Drivebase::new(
        [(19, true), (14, true), (17, true)],
        [(5, false), (4, false), (3, false)],
        MotorControl::BrakeBrake,
        254.0,
        75.0,
    );

    let mut imu = imu::Imu::new(7);
    let mut imu_pid = pid::Pid::new(0.55, 0.055, 2.2);

    let mut track_pid = false;
    let ramsete = Ramsete::new(0.3, 0.3);
    let ramsete_path = RamsetePath::new(
        vec![
            (Vec2::new(0.00, 10.00), 1.58),
            (Vec2::new(-0.03, 17.62), 1.58),
            (Vec2::new(-0.13, 25.62), 1.59),
            (Vec2::new(-0.30, 33.99), 1.60),
            (Vec2::new(-0.53, 42.72), 1.60),
            (Vec2::new(-0.82, 51.82), 1.61),
            (Vec2::new(-1.16, 61.27), 1.61),
            (Vec2::new(-1.56, 71.06), 1.62),
            (Vec2::new(-2.02, 81.18), 1.62),
            (Vec2::new(-2.53, 91.64), 1.62),
            (Vec2::new(-3.09, 102.42), 1.63),
            (Vec2::new(-3.70, 113.51), 1.63),
            (Vec2::new(-4.35, 124.92), 1.63),
            (Vec2::new(-5.05, 136.62), 1.63),
            (Vec2::new(-5.79, 148.62), 1.63),
            (Vec2::new(-6.57, 160.90), 1.64),
            (Vec2::new(-7.39, 173.47), 1.64),
            (Vec2::new(-8.25, 186.30), 1.64),
            (Vec2::new(-9.14, 199.40), 1.64),
            (Vec2::new(-10.06, 212.76), 1.64),
            (Vec2::new(-11.01, 226.37), 1.64),
            (Vec2::new(-11.98, 240.22), 1.64),
            (Vec2::new(-12.99, 254.31), 1.64),
            (Vec2::new(-14.02, 268.63), 1.64),
            (Vec2::new(-15.07, 283.17), 1.64),
            (Vec2::new(-16.14, 297.92), 1.64),
            (Vec2::new(-17.22, 312.88), 1.64),
            (Vec2::new(-18.33, 328.04), 1.64),
            (Vec2::new(-19.44, 343.40), 1.64),
            (Vec2::new(-20.57, 358.94), 1.64),
            (Vec2::new(-21.71, 374.66), 1.64),
            (Vec2::new(-22.85, 390.55), 1.64),
            (Vec2::new(-24.00, 406.60), 1.64),
            (Vec2::new(-25.16, 422.81), 1.64),
            (Vec2::new(-26.31, 439.17), 1.64),
            (Vec2::new(-27.47, 455.68), 1.64),
            (Vec2::new(-28.62, 472.32), 1.64),
            (Vec2::new(-29.76, 489.09), 1.64),
            (Vec2::new(-30.90, 505.97), 1.64),
            (Vec2::new(-32.04, 522.98), 1.64),
            (Vec2::new(-33.16, 540.09), 1.64),
            (Vec2::new(-34.26, 557.30), 1.63),
            (Vec2::new(-35.36, 574.60), 1.63),
            (Vec2::new(-36.43, 591.99), 1.63),
            (Vec2::new(-37.49, 609.45), 1.63),
            (Vec2::new(-38.52, 626.99), 1.63),
            (Vec2::new(-39.54, 644.59), 1.63),
            (Vec2::new(-40.52, 662.24), 1.62),
            (Vec2::new(-41.48, 679.95), 1.62),
            (Vec2::new(-42.41, 697.69), 1.62),
            (Vec2::new(-43.31, 715.47), 1.62),
            (Vec2::new(-44.18, 733.28), 1.62),
            (Vec2::new(-45.01, 751.11), 1.62),
            (Vec2::new(-45.80, 768.95), 1.61),
            (Vec2::new(-46.56, 786.80), 1.61),
            (Vec2::new(-47.27, 804.65), 1.61),
            (Vec2::new(-47.94, 822.49), 1.61),
            (Vec2::new(-48.56, 840.31), 1.60),
            (Vec2::new(-49.14, 858.12), 1.60),
            (Vec2::new(-49.67, 875.89), 1.60),
            (Vec2::new(-50.14, 893.62), 1.59),
            (Vec2::new(-50.56, 911.31), 1.59),
            (Vec2::new(-50.93, 928.95), 1.59),
            (Vec2::new(-51.24, 946.53), 1.58),
            (Vec2::new(-51.48, 964.04), 1.58),
            (Vec2::new(-51.67, 981.48), 1.58),
            (Vec2::new(-51.79, 998.84), 1.57),
            (Vec2::new(-51.85, 1016.11), 1.57),
            (Vec2::new(-51.84, 1033.29), 1.57),
            (Vec2::new(-51.75, 1050.37), 1.56),
            (Vec2::new(-51.60, 1067.34), 1.56),
            (Vec2::new(-51.37, 1084.19), 1.55),
            (Vec2::new(-51.07, 1100.92), 1.55),
            (Vec2::new(-50.69, 1117.51), 1.54),
            (Vec2::new(-50.23, 1133.97), 1.54),
            (Vec2::new(-49.68, 1150.29), 1.53),
            (Vec2::new(-49.05, 1166.45), 1.53),
            (Vec2::new(-48.34, 1182.45), 1.52),
            (Vec2::new(-47.54, 1198.29), 1.51),
            (Vec2::new(-46.64, 1213.95), 1.51),
            (Vec2::new(-45.66, 1229.43), 1.50),
            (Vec2::new(-44.58, 1244.72), 1.49),
            (Vec2::new(-43.40, 1259.82), 1.49),
            (Vec2::new(-42.12, 1274.72), 1.48),
            (Vec2::new(-40.75, 1289.40), 1.47),
            (Vec2::new(-39.27, 1303.87), 1.46),
            (Vec2::new(-37.69, 1318.12), 1.45),
            (Vec2::new(-36.00, 1332.13), 1.44),
            (Vec2::new(-34.20, 1345.90), 1.43),
            (Vec2::new(-32.29, 1359.43), 1.42),
            (Vec2::new(-30.27, 1372.71), 1.41),
            (Vec2::new(-28.13, 1385.72), 1.40),
            (Vec2::new(-25.88, 1398.47), 1.38),
            (Vec2::new(-23.50, 1410.95), 1.37),
            (Vec2::new(-21.01, 1423.14), 1.35),
            (Vec2::new(-18.40, 1435.04), 1.34),
            (Vec2::new(-15.65, 1446.65), 1.32),
            (Vec2::new(-12.79, 1457.96), 1.30),
            (Vec2::new(-9.79, 1468.95), 1.29),
            (Vec2::new(-6.66, 1479.63), 1.27),
            (Vec2::new(0.00, 1500.00), 1.22),
            (Vec2::new(3.57, 1509.78), 1.20),
            (Vec2::new(7.33, 1519.41), 1.18),
            (Vec2::new(11.30, 1528.91), 1.15),
            (Vec2::new(15.46, 1538.26), 1.13),
            (Vec2::new(19.80, 1547.47), 1.11),
            (Vec2::new(24.33, 1556.55), 1.09),
            (Vec2::new(29.04, 1565.49), 1.06),
            (Vec2::new(33.93, 1574.29), 1.04),
            (Vec2::new(38.99, 1582.96), 1.02),
            (Vec2::new(44.22, 1591.50), 1.00),
            (Vec2::new(49.61, 1599.91), 0.98),
            (Vec2::new(55.16, 1608.18), 0.96),
            (Vec2::new(60.87, 1616.33), 0.94),
            (Vec2::new(66.73, 1624.36), 0.92),
            (Vec2::new(72.74, 1632.25), 0.90),
            (Vec2::new(78.90, 1640.03), 0.88),
            (Vec2::new(85.19, 1647.68), 0.86),
            (Vec2::new(91.62, 1655.21), 0.85),
            (Vec2::new(98.18, 1662.62), 0.83),
            (Vec2::new(104.87, 1669.92), 0.81),
            (Vec2::new(111.69, 1677.09), 0.79),
            (Vec2::new(118.62, 1684.15), 0.78),
            (Vec2::new(125.67, 1691.10), 0.76),
            (Vec2::new(132.83, 1697.94), 0.75),
            (Vec2::new(140.10, 1704.66), 0.73),
            (Vec2::new(147.47, 1711.28), 0.72),
            (Vec2::new(154.94, 1717.79), 0.70),
            (Vec2::new(162.51, 1724.19), 0.69),
            (Vec2::new(170.17, 1730.48), 0.67),
            (Vec2::new(177.91, 1736.68), 0.66),
            (Vec2::new(185.74, 1742.77), 0.65),
            (Vec2::new(193.65, 1748.76), 0.64),
            (Vec2::new(201.64, 1754.65), 0.62),
            (Vec2::new(209.69, 1760.44), 0.61),
            (Vec2::new(217.82, 1766.13), 0.60),
            (Vec2::new(226.00, 1771.73), 0.59),
            (Vec2::new(234.25, 1777.24), 0.58),
            (Vec2::new(242.55, 1782.65), 0.57),
            (Vec2::new(250.90, 1787.98), 0.56),
            (Vec2::new(259.30, 1793.21), 0.55),
            (Vec2::new(267.74, 1798.36), 0.54),
            (Vec2::new(276.23, 1803.42), 0.53),
            (Vec2::new(284.74, 1808.39), 0.52),
            (Vec2::new(293.29, 1813.28), 0.51),
            (Vec2::new(301.86, 1818.09), 0.50),
            (Vec2::new(310.46, 1822.82), 0.49),
            (Vec2::new(319.07, 1827.46), 0.49),
            (Vec2::new(327.70, 1832.03), 0.48),
            (Vec2::new(336.34, 1836.53), 0.47),
            (Vec2::new(344.99, 1840.94), 0.47),
            (Vec2::new(353.64, 1845.29), 0.46),
            (Vec2::new(362.29, 1849.56), 0.45),
            (Vec2::new(370.93, 1853.76), 0.45),
            (Vec2::new(379.56, 1857.89), 0.44),
            (Vec2::new(388.18, 1861.96), 0.43),
            (Vec2::new(396.79, 1865.95), 0.43),
            (Vec2::new(405.37, 1869.88), 0.42),
            (Vec2::new(413.92, 1873.75), 0.42),
            (Vec2::new(422.44, 1877.56), 0.42),
            (Vec2::new(430.94, 1881.30), 0.41),
            (Vec2::new(439.39, 1884.99), 0.41),
            (Vec2::new(447.80, 1888.61), 0.40),
            (Vec2::new(456.16, 1892.18), 0.40),
            (Vec2::new(464.48, 1895.70), 0.40),
            (Vec2::new(472.74, 1899.16), 0.39),
            (Vec2::new(480.94, 1902.57), 0.39),
            (Vec2::new(489.08, 1905.93), 0.39),
            (Vec2::new(497.16, 1909.24), 0.39),
            (Vec2::new(505.16, 1912.50), 0.39),
            (Vec2::new(513.09, 1915.71), 0.38),
            (Vec2::new(520.94, 1918.88), 0.38),
            (Vec2::new(528.71, 1922.01), 0.38),
            (Vec2::new(536.39, 1925.09), 0.38),
            (Vec2::new(543.99, 1928.13), 0.38),
            (Vec2::new(551.48, 1931.13), 0.38),
            (Vec2::new(558.88, 1934.10), 0.38),
            (Vec2::new(566.18, 1937.02), 0.38),
            (Vec2::new(573.37, 1939.92), 0.38),
            (Vec2::new(580.45, 1942.78), 0.39),
            (Vec2::new(587.41, 1945.60), 0.39),
            (Vec2::new(594.26, 1948.40), 0.39),
            (Vec2::new(600.98, 1951.16), 0.39),
            (Vec2::new(607.58, 1953.90), 0.40),
            (Vec2::new(614.04, 1956.61), 0.40),
            (Vec2::new(620.37, 1959.30), 0.41),
            (Vec2::new(626.56, 1961.96), 0.41),
            (Vec2::new(632.61, 1964.60), 0.42),
            (Vec2::new(638.51, 1967.22), 0.42),
            (Vec2::new(644.26, 1969.82), 0.43),
            (Vec2::new(649.86, 1972.40), 0.44),
            (Vec2::new(655.29, 1974.96), 0.45),
            (Vec2::new(660.57, 1977.51), 0.46),
            (Vec2::new(665.67, 1980.04), 0.47),
            (Vec2::new(670.60, 1982.57), 0.49),
            (Vec2::new(675.36, 1985.08), 0.50),
            (Vec2::new(679.94, 1987.58), 0.52),
            (Vec2::new(684.34, 1990.08), 0.53),
            (Vec2::new(688.54, 1992.56), 0.55),
            (Vec2::new(692.56, 1995.05), 0.58),
        ],
        ramsete,
    );

    // example path
    let mut path = path!(
        //Nop {},
        //TimedSegment::new(Box::new(Nop {}), Duration::from_millis(200)),
        ramsete_path /*RamsetePoint::new(
                         (Vec2::new(-350.0, -350.0), std::f64::consts::FRAC_PI_4),
                         ramsete
                     ),*/
    );

    //let mut drivebase_measurer = drivebase_measurer::DriveBaseMeasurer::new(75.0);
    //let mut odometry = odometry::Odometry::new(5);
    let mut odom = odometry::Odom::new(Vec2::new(0.0, 0.0), 90.0f64.to_radians(), &imu, &drivebase);

    let mut latch_close: i8 = 0;
    let mut at_volt = false;

    loop {
        let (pkt, just_updated) = brain.update_state(&mut controller);
        let pkt_to_write = brain.get_brain_pkt();

        // update imu, odometry stuff
        imu.update(&pkt);
        drivebase.update(&pkt);
        odom.update(&imu, &drivebase);

        match path.follow(&odom, &mut imu_pid) {
            PathOutput::Voltages(_) => {
                if at_volt == false {
                    log::info!("switch to voltages");
                }
                at_volt = true;
                drivebase.write_powers(controller.ly(), -controller.rx(), pkt_to_write);
            }
            PathOutput::LinearAngularVelocity(la) => {
                if just_updated {
                    log::info!("{la:?}");
                }
                drivebase.write_linear_angular_vel(la.x, la.y, pkt_to_write);
            }
        }

        if just_updated {}

        // intake
        if controller.pressed(RIGHT_TRIGGER_2) {
            latch_close = 1;
        }
        if controller.pressed(RIGHT_TRIGGER_1) {
            latch_close = 2;
        }
        if controller.pressed(X) {
            latch_close = 0;
        }

        if latch_close == 2 {
            pkt_to_write.set_motors[9] = MotorControl::Voltage(-4.0);
        } else if latch_close == 1 {
            pkt_to_write.set_motors[9] = MotorControl::Voltage(4.0);
        } else {
            pkt_to_write.set_motors[9] = MotorControl::BrakeHold;
        }

        //
        brain.write_changes();
    }
}
