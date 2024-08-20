use plotly::Histogram;
use plotly::Plot;
use robot_serial::protocol;

fn main() {
    env_logger::init();

    const TEST_SIZE: usize = 485;
    let pkt = protocol::ToBrain::Ping(vec![0; TEST_SIZE]);
    let vec = postcard::to_stdvec_cobs(&pkt).unwrap();
    log::info!(
        "encoded test packet size: {} from size {TEST_SIZE}",
        vec.len()
    );

    log::info!("waiting for brain connection");
    let mut bm;
    loop {
        if let Some(v) = robot_serial::BrainMediator::new() {
            bm = v;
            log::info!("brain connected");
            break;
        }
    }

    let mut sent = false;
    let mut now = std::time::Instant::now();
    let mut samples = Vec::new();

    loop {
        if samples.len() > 1000 {
            break;
        }
        if !sent {
            if let Err(e) = bm.try_write(&pkt) {
                log::warn!("{e:?}");
            } else {
                sent = true;
                now = std::time::Instant::now();
            }
        }
        match bm.try_read() {
            Ok(v) => {
                for e in &v {
                    match e {
                        protocol::ToRobot::Pong(_) => {
                            samples.push(now.elapsed().as_micros());
                            log::info!(
                                "round trip of size {TEST_SIZE} (encoded size {}) done in: {}ms",
                                vec.len(),
                                now.elapsed().as_millis()
                            );
                            sent = false;
                        }
                        v => log::info!("got other: {v:?}"),
                    }
                }
            }
            Err(e) => log::warn!("{e:?}"),
        }
    }
    let mut plot = Plot::new();
    let trace = Histogram::new(samples).name(format!(
        "round trip time with an encoded packet size of {}",
        vec.len()
    ));
    plot.add_trace(trace);
    plot.show();
}
