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
use clap::{App, Arg, SubCommand};
use std::path::{Path, PathBuf};

fn main() {
    let matches = App::new("datagen")
        .version("0.1")
        .author("Matt Jadczak <mnj24@cam.ac.uk>")
        .about("Generates trajectory data for simulated robots")
        .subcommand(SubCommand::with_name("test")
            .about("Tests simple trajectory evaluation"))
        .subcommand(SubCommand::with_name("gen-traj")
            .about("Generates reference trajectories")
            .arg(Arg::with_name("length")
                .short("l")
                .long("length")
                .takes_value(true)
                .help("Sets the (minimum) length of the generated trajectory, in seconds"))
            .arg(Arg::with_name("variability")
                .short("v")
                .long("variability")
                .takes_value(true)
                .help("Sets variability of the trajectory"))
            .arg(Arg::with_name("rsd")
                .short("r")
                .long("rsd")
                .takes_value(true)
                .help("Sets rsd of the trajectory"))
            .arg(Arg::with_name("num")
                .short("n")
                .long("num-trajectories")
                .takes_value(true)
                .help("How many trajectories to generate")
                .required(true))
            .arg(Arg::with_name("output_dir")
                .short("o")
                .long("output-dir")
                .takes_value(true)
                .help("Output directory to place generated trajectories in")
                .required(true))
        )
        .subcommand(SubCommand::with_name("gen-data")
            .about("Generates trajectory data")
            .arg(Arg::with_name("length")
                .short("l")
                .long("length")
                .takes_value(true)
                .help("Sets the (minimum) length of the generated trajectory, in seconds"))
            .arg(Arg::with_name("variability")
                .short("v")
                .long("variability")
                .takes_value(true)
                .help("Sets variability of the trajectory"))
            .arg(Arg::with_name("rsd")
                .short("r")
                .long("rsd")
                .takes_value(true)
                .help("Sets rsd of the trajectory"))
            .arg(Arg::with_name("num")
                .short("n")
                .long("num-trajectories")
                .takes_value(true)
                .help("How many trajectories to generate")
                .required(true))
            .arg(Arg::with_name("output_dir")
                .short("o")
                .long("output-dir")
                .takes_value(true)
                .help("Output directory to place generated trajectories in")
                .required(true))
        )
        .get_matches();

    match matches.subcommand() {
        ("test", _) => test_traj_gen(),
        ("gen-traj", Some(m)) => {
            let length = m.value_of("length").map_or(10., |s| s.parse::<f64>().unwrap());
            let variability = m.value_of("variability").map_or(2., |s| s.parse::<f64>().unwrap());
            let rsd = m.value_of("rsd").map_or(0.1, |s| s.parse::<f64>().unwrap());
            let num = m.value_of("num").unwrap().parse::<usize>().unwrap();
            let out = m.value_of("output_dir").unwrap();
            traj_gen(length, variability, rsd, num, out);
        },
        ("gen-data", Some(m)) => {
            let length = m.value_of("length").map_or(10., |s| s.parse::<f64>().unwrap());
            let variability = m.value_of("variability").map_or(2., |s| s.parse::<f64>().unwrap());
            let rsd = m.value_of("rsd").map_or(0.1, |s| s.parse::<f64>().unwrap());
            let num = m.value_of("num").unwrap().parse::<usize>().unwrap();
            let out = m.value_of("output_dir").unwrap();
            data_gen(length, variability, rsd, num, out);
        }
        _ => eprintln!("Command needs to be specified. Use `--help` to view usage.")
    }
}

fn traj_gen(length: f64, variability: f64, rsd: f64, num: usize, out: &str) {
    let max_speed = 0.5;
    let num_len = num.to_string().len();
    let out_dir_path = Path::new(out);
    std::fs::create_dir_all(out_dir_path).unwrap();

    for i in 0..num {
        print!("\rWorking... [{:0width$}/{:0width$}]", i+1, num, width = num_len);
        let trajectory = trajectory::generate_1d_trajectory_points_simple(max_speed, length, variability, rsd);

        // write to a test path
        let file_name = format!("traj_{:0width$}.csv", i, width = num_len);
        let mut writer = csv::Writer::from_path(out_dir_path.join(&file_name)).unwrap();
        writer.write_record(&["t", "x"]).unwrap();
        for &(t, r) in trajectory.iter() {
            writer.write_record(&[t.to_string(), r.to_string()]).unwrap();
        }
        writer.flush().unwrap();
    }
    println!("\nDone!");
}

fn data_gen(length: f64, variability: f64, rsd: f64, num: usize, out: &str) {
    let max_speed = 0.5;
    let num_len = num.to_string().len();
    let out_dir_path = Path::new(out);
    std::fs::create_dir_all(out_dir_path).unwrap();

    for i in 0..num {
        print!("\rWorking... [{:0width$}/{:0width$}]", i+1, num, width = num_len);
        let trajectory = trajectory::generate_1d_trajectory_points_simple(max_speed, length, variability, rsd);
        let resolution = 1. / 10.;
        let trajectory_mode = simulation::LeaderTrajectoryMode::Follow;
        let converted_trajectory = trajectory::NaiveTrajectory::from_points(resolution, trajectory);

        // simulate
        let cparams = simulation::PControllerParams::default();
        let controllers = vec![
            simulation::PController::new(cparams),
            simulation::PController::new(cparams)
        ];
        let formation = simulation::SimpleFormation::new(2, 0., vec![0.2, 0.]);
        let simulation0 =
            simulation::SimpleSimulation::new(2, 0, controllers.clone(), &formation,
                                              &converted_trajectory, trajectory_mode);
        let simulation1 =
            simulation::SimpleSimulation::new(2, 1, controllers, &formation,
                                              &converted_trajectory, trajectory_mode);
        let mut observer = simulation::SimpleObserver::new(0.05);
        //let mut observer = simulation::PerfectObserver {};
        let result0 = simulation0.run(10., resolution, &mut observer).into_data();
        let result1 = simulation1.run(10., resolution, &mut observer).into_data();

        let file_name = format!("traj_{:0width$}_l0.csv", i, width = num_len);
        let mut writer = csv::Writer::from_path(out_dir_path.join(&file_name)).unwrap();
        writer.write_record(&["t", "r1", "r2"]).unwrap();
        for (i, (r1, r2)) in result0[0].iter().zip(result0[1].iter()).enumerate() {
            let t = i as f64 * resolution;
            writer.write_record(&[t.to_string(), r1.to_string(), r2.to_string()]).unwrap();
        }
        writer.flush().unwrap();

        let file_name = format!("traj_{:0width$}_l1.csv", i, width = num_len);
        let mut writer = csv::Writer::from_path(out_dir_path.join(&file_name)).unwrap();
        writer.write_record(&["t", "r1", "r2"]).unwrap();
        for (i, (r1, r2)) in result1[0].iter().zip(result1[1].iter()).enumerate() {
            let t = i as f64 * resolution;
            writer.write_record(&[t.to_string(), r1.to_string(), r2.to_string()]).unwrap();
        }
        writer.flush().unwrap();
    }
    println!("\nDone!");
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
        (11., 2.)
    ];

    let resolution = 1. / 10.;
    let leader_id = 1;
    let trajectory_mode = simulation::LeaderTrajectoryMode::Follow;

    // convert
    let converted_trajectory = trajectory::NaiveTrajectory::from_points(resolution, sample_trajectory_points);

    // simulate
    let cparams = simulation::PIDControllerParams::default();
    let controllers = vec![
        simulation::PIDController::new(cparams),
        simulation::PIDController::new(cparams)
    ];
    let formation = simulation::SimpleFormation::new(2, 0., vec![0.2, 0.]);
    let simulation =
        simulation::SimpleSimulation::new(2, leader_id, controllers, &formation,
                                          &converted_trajectory, trajectory_mode);
    //let mut observer = simulation::SimpleObserver::new(0.1);
    let mut observer = simulation::PerfectObserver {};

    let result = simulation.run(10., resolution, &mut observer).into_data();

    // write to a test path
    let mut writer = csv::Writer::from_path("../test_data/test_traj.csv").unwrap();
    writer.write_record(&["time", "r1", "r2"]).unwrap();
    for (i, (r1, r2)) in result[0].iter().zip(result[1].iter()).enumerate() {
        let t = i as f64 * resolution;
        writer.write_record(&[t.to_string(), r1.to_string(), r2.to_string()]).unwrap();
    }
    writer.flush().unwrap();
}