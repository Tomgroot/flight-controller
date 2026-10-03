use clap::Parser;
use fc_core::FlightController;
use sim::{Imu, ImuParams, Quad, QuadParams, RcSource};
use std::path::PathBuf;
use std::time::{Duration, Instant};

const DT: f32 = 0.001;
const LOG_EVERY: u64 = 10;

/// Fly fc-core against the quad simulator in lockstep.
#[derive(Parser)]
struct Args {
    /// Simulated seconds to run.
    #[arg(long, default_value_t = 10.0)]
    duration: f32,
    /// Throttle to wall-clock time instead of running as fast as possible.
    #[arg(long)]
    realtime: bool,
    /// Write a .rrd file instead of spawning the Rerun viewer.
    #[arg(long)]
    save: Option<PathBuf>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    let builder = rerun::RecordingStreamBuilder::new("flight-controller-sitl");
    let rec = match &args.save {
        Some(path) => builder.save(path)?,
        None => builder.spawn()?,
    };

    let mut quad = Quad::new(QuadParams::default());
    let mut imu = Imu::new(ImuParams::default(), 42);
    let rc_source = RcSource;
    let mut fc = FlightController::new();

    rec.log_static("world", &rerun::ViewCoordinates::RIGHT_HAND_Z_UP())?;
    rec.log_static(
        "world/quad",
        &rerun::Boxes3D::from_half_sizes([(0.16, 0.16, 0.03)]),
    )?;

    let start = Instant::now();
    let steps = (args.duration / DT) as u64;
    for step in 0..steps {
        let t = step as f32 * DT;
        let rc = rc_source.sample(t);
        let imu_sample = imu.sample(&quad);
        let motors = fc.update(&imu_sample, &rc, DT);
        quad.step(DT, &motors);

        if args.realtime {
            let target = Duration::from_secs_f32(t + DT);
            if let Some(wait) = target.checked_sub(start.elapsed()) {
                std::thread::sleep(wait);
            }
        }

        if step % LOG_EVERY != 0 {
            continue;
        }
        rec.set_duration_secs("sim_time", t);
        let a = quad.attitude;
        rec.log(
            "world/quad",
            &rerun::Transform3D::from_translation_rotation(
                quad.position.to_array(),
                rerun::Quaternion::from_xyzw([a.x, a.y, a.z, a.w]),
            ),
        )?;
        for (i, name) in ["x", "y", "z"].iter().enumerate() {
            rec.log(
                format!("imu/gyro/{name}"),
                &rerun::Scalars::single(imu_sample.gyro[i] as f64),
            )?;
            rec.log(
                format!("imu/accel/{name}"),
                &rerun::Scalars::single(imu_sample.accel[i] as f64),
            )?;
            rec.log(
                format!("state/position/{name}"),
                &rerun::Scalars::single(quad.position[i] as f64),
            )?;
        }
        for (i, value) in motors.0.iter().enumerate() {
            rec.log(format!("motors/{i}"), &rerun::Scalars::single(*value as f64))?;
        }
        for (i, name) in ["roll", "pitch", "throttle", "yaw", "arm"].iter().enumerate() {
            rec.log(format!("rc/{name}"), &rerun::Scalars::single(rc.0[i] as f64))?;
        }
    }

    println!(
        "simulated {:.1} s in {:.2} s, final position {:?}",
        args.duration,
        start.elapsed().as_secs_f32(),
        quad.position
    );
    Ok(())
}
