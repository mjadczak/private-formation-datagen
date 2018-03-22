#[macro_use]
extern crate clap;
extern crate pid_control;
extern crate num;
extern crate rand;
extern crate csv;

pub mod simulation;
pub mod trajectory;
pub mod base;

use trajectory::UniformResolutionTrajectory;
use simulation::{Simulation, SimulationResult};

fn main() {
    let matches = clap_app!(datagen =>
        (version: "0.1")
        (author: "Matt Jadczak <mnj24@cam.ac.uk>")
        (about: "Generates trajectory data for simulated robots")
        (@subcommand test =>
            (about: "Tests trajectory generation")
        )
    ).get_matches();

    match matches.subcommand_name() {
        Some("test") => test_traj_gen(),
        _ => eprintln!("Command needs to be specified. Use `--help` to view usage.")
    }
}

fn test_traj_gen() {
    // Assume 0.5m/s speed
    let sample_trajectory_points = vec![
        // max speed for 3s
        (0., 0.),
        (3., 1.5),
        // stop for 2s
        (5., 1.5),
        // half speed for 2s
        (7., 2.),
        (10., 2.)
    ];
    println!("Sample trajectory piecewise: {:?}", sample_trajectory_points);

    let resolution = 0.1;

    // convert
    let converted_trajectory = trajectory::NaiveTrajectory::from_points(resolution, sample_trajectory_points);

    // simulate
    let cparams = simulation::PControllerParams::default();
    let controllers = vec![
        simulation::PController::new(cparams),
        simulation::PController::new(cparams)
    ];
    let formation = simulation::SimpleFormation::new(2, 0., vec![0.2, 0.]);
    let simulation =
        simulation::SimpleSimulation::new(2, 0, controllers, &formation,
                                          &converted_trajectory, simulation::LeaderFollowMode::FollowTrajectory);
    let result = simulation.run(10., 0.1).into_data();

    // write to a test path
    let mut writer = csv::Writer::from_path("../test_data/test_traj.csv").unwrap();
    writer.write_record(&["time", "r1", "r2"]);
    for (i, (r1, r2)) in result[0].iter().zip(result[1].iter()).enumerate() {
        let t = i as f64 * resolution;
        writer.write_record(&[t.to_string(), r1.to_string(), r2.to_string()]).unwrap();
    }
    writer.flush().unwrap();
}