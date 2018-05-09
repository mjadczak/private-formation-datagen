use base::*;
use failure::Error;
use rand::distributions::{Distribution, Normal, Range, StandardNormal, Uniform};
use rand::{FromEntropy, Rng, SmallRng};
use simulation;
use simulation::Simulation;
use slugify::slugify;
use std;
use std::collections::HashMap;
use std::fs::File;
use std::path::PathBuf;
use tf_record;
use time;
use trajectory;
use serde_yaml;
use std::io::Write;
use simulation::SimulationResult;

type Result<R> = ::std::result::Result<R, Error>;
type Params = HashMap<String, ConstantParam>;
type TrajectorySets<S> = Vec<(Params, Vec<trajectory::NaiveTrajectory<S>>)>;

#[derive(Debug, Clone, Deserialize)]
pub struct ScenarioSpec {
    pub name: String,
    pub working_dir: String,
    pub slug: Option<String>,
    pub robot: RobotSpec,
    pub observer: ObserverSpec,
    pub resolution: f64,
    pub length: f64,
    pub dimensions: usize,
    pub formations: FormationSpec,
    pub reference_trajectories: ReferenceTrajectorySpec,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case", tag = "type")]
pub enum ObserverSpec {
    Simple { error: f64 },
    Perfect {},
}

impl ObserverSpec {
    fn get_observer(&self) -> simulation::SimpleObserver {
        match *self {
            ObserverSpec::Perfect {} => simulation::SimpleObserver::new(0.),
            ObserverSpec::Simple { error } => simulation::SimpleObserver::new(error),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct RobotSpec {
    pub max_speed: f64,
    pub controller: ControllerSpec,
    pub num_robots: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type")]
pub enum ControllerSpec {
    PID {
        p_gain: f64,
        i_gain: f64,
        d_gain: f64,
    },
}

impl Default for ControllerSpec {
    fn default() -> Self {
        ControllerSpec::PID {
            p_gain: 0.,
            i_gain: 0.,
            d_gain: 0.,
        }
    }
}

impl ControllerSpec {
    pub fn get_1d_controller(&self, max_speed: f64) -> simulation::PIDController {
        let ControllerSpec::PID {
            p_gain,
            i_gain,
            d_gain,
        } = *self;
        let params = simulation::PIDControllerParams {
            p_gain,
            d_gain,
            i_gain,
            vel_limits: (-max_speed, max_speed),
        };
        simulation::PIDController::new(params)
    }

    pub fn get_2d_controller(&self, max_speed: f64) -> simulation::UniformPIDController2D {
        let ControllerSpec::PID {
            p_gain,
            i_gain,
            d_gain,
        } = *self;
        let params = simulation::PIDControllerParams {
            p_gain,
            d_gain,
            i_gain,
            vel_limits: (-max_speed, max_speed),
        };
        simulation::UniformPIDController2D::new(params)
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct FormationSpec {
    pub generator: String,
    pub params: ParamsSpec,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case", tag = "type")]
pub enum ParamsSpec {
    Random {
        values: HashMap<String, RandomParamSpec>,
    },
    Constant {
        values: HashMap<String, ConstantParam>,
    },
}

impl ParamsSpec {
    fn specialise<R: Rng + ?Sized>(&self, rng: &mut R) -> Params {
        match *self {
            ParamsSpec::Constant { ref values } => values.clone(),
            ParamsSpec::Random { ref values } => values
                .iter()
                .map(|el| Self::specialise_random(el, rng))
                .collect(),
        }
    }

    fn specialise_random<R: Rng + ?Sized>(
        el: (&String, &RandomParamSpec),
        rng: &mut R,
    ) -> (String, ConstantParam) {
        let (key, spec) = el;
        let key = key.to_string();
        match *spec {
            RandomParamSpec::Uniform { range } => {
                let (lower, upper) = range;
                let val = Range::sample_single(lower, upper, rng);
                (key, ConstantParam::Float(val))
            }
            RandomParamSpec::Normal { range } => {
                let (lower, upper) = range;
                let mean = (lower + upper) / 2.;
                let sd = (mean - lower) / 1.96;
                let sample: f64 = rng.sample(StandardNormal);
                let val1 = (sample * sd) + mean;
                let val = val1.max(lower).min(upper);
                trace!("Specialising param with range {:?}: mean={}, sd={}, val={}, result={}", range, mean, sd, val1, val);
                (key, ConstantParam::Float(val))
            }
            RandomParamSpec::Constant { value } => (key, ConstantParam::Float(value)),
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case", tag = "dist")]
pub enum RandomParamSpec {
    Uniform { range: (f64, f64) },
    Normal { range: (f64, f64) },
    Constant { value: f64 },
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(untagged)]
pub enum ConstantParam {
    Float(f64),
    Int(i64),
}

impl ConstantParam {
    fn as_f64(&self) -> Result<f64> {
        if let ConstantParam::Float(val) = *self {
            Ok(val)
        } else {
            bail!("value is not a float")
        }
    }
}

#[derive(Debug, Clone, Deserialize)]
pub struct ReferenceTrajectorySpec {
    pub num_sets: usize,
    pub num_per_set: usize,
    pub generator: String,
    pub params: ParamsSpec,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct DatasetDescription {
    pub num_robots: usize,
    pub dimensions: usize,
    pub points_per_trajectory: usize,
    pub resolution: f64,
    pub features: Params,
    pub controller: ControllerSpec,
    pub files: Vec<DataFileDescription>,
}

#[derive(Debug, Clone, Default, Serialize)]
pub struct DataFileDescription {
    pub file: String,
    pub num_trajectories: usize,
    pub features: Params,
    pub per_trajectory_features: Vec<Params>,
}

#[derive(Default, Debug)]
struct ScenarioExecutionContext {
    working_dir: PathBuf,
    slug: String,
    info_file_path: PathBuf,
    data_dir: PathBuf,
    description: DatasetDescription,
}

impl ScenarioExecutionContext {
    pub fn new() -> Self {
        Default::default()
    }

    pub fn into_info_file_path(self) -> PathBuf {
        self.info_file_path
    }

    pub fn execute(&mut self, spec: &ScenarioSpec) -> Result<()> {
        self.working_dir = PathBuf::from(&spec.working_dir);
        self.slug =
            if let Some(ref slug) = spec.slug {
                slug.clone()
            } else {
                Self::get_prefix() + &slugify(&spec.name, "", "_", None)
            };
        self.info_file_path = self.working_dir.join(&self.slug);
        self.info_file_path.set_extension("dginfo.yaml");
        self.data_dir = self.working_dir.join(self.slug.clone() + "-data");

        debug!("Working dir is {:?}, data dir is {:?}", self.working_dir, self.data_dir);

        std::fs::create_dir_all(&self.data_dir)?;

        self.description.num_robots = spec.robot.num_robots;
        self.description.dimensions = spec.dimensions;
        self.description.points_per_trajectory = (spec.length / spec.resolution) as usize + 1;
        self.description.resolution = spec.resolution;
        self.description.controller = spec.robot.controller.clone();

        ensure!(
            spec.robot.num_robots == 2,
            "currently only 2 robots are supported"
        );
        ensure!(
            spec.reference_trajectories.generator == "simple",
            "only simple trajectory generation supported"
        );
        let num_sets = spec.reference_trajectories.num_sets;
        let num_per_set = spec.reference_trajectories.num_per_set;
        let mut rng = SmallRng::from_entropy();
        match spec.dimensions {
            1 => {
                let mut trajectory_sets: TrajectorySets<Metres> = Vec::with_capacity(num_sets);
                for _ in 0..num_sets {
                    let params = spec.reference_trajectories.params.specialise(&mut rng);
                    let generator = TrajectoryGeneratorSimple1D::from_params(&params)?;
                    let mut set: Vec<trajectory::NaiveTrajectory<Metres>> =
                        Vec::with_capacity(num_per_set);
                    for _ in 0..num_per_set {
                        set.push(generator.generate(
                            spec.length,
                            spec.resolution,
                            spec.robot.max_speed,
                        ));
                    }
                    trajectory_sets.push((params, set));
                }

                let controller = spec.robot
                    .controller
                    .get_1d_controller(spec.robot.max_speed);

                let generator = Simple1DFormationGenerator::new(&spec.formations.params)?;

                let sensor = simulation::SharpIrSensor::new();

                let observer = spec.observer.get_observer();

                self.stage2(spec, trajectory_sets, controller, generator, sensor, observer)
            }
            2 => {
                let mut trajectory_sets: TrajectorySets<Metres2D> = Vec::with_capacity(num_sets);
                for _ in 0..num_sets {
                    let params = spec.reference_trajectories.params.specialise(&mut rng);
                    let generator = TrajectoryGeneratorSimple2D::from_params(&params)?;
                    let mut set: Vec<trajectory::NaiveTrajectory<Metres2D>> =
                        Vec::with_capacity(num_per_set);
                    for _ in 0..num_per_set {
                        set.push(generator.generate(
                            spec.length,
                            spec.resolution,
                            spec.robot.max_speed,
                        ));
                    }
                    trajectory_sets.push((params, set));
                }

                let controller = spec.robot
                    .controller
                    .get_2d_controller(spec.robot.max_speed);

                // todo formation generator

                let sensor = simulation::CombinedIrEncoderSensor::new();

                let observer = spec.observer.get_observer();

                self.stage2(spec, trajectory_sets, controller, Dummy2DFormationGenerator, sensor, observer)
            }
            _ => bail!("Only 1D and 2D operation supported"),
        }
    }

    fn stage2<
        S: Vector,
        C: simulation::Controller<S>,
        F: simulation::Formation<S>,
        G: FormationGenerator<S, Result=F>,
        Se: simulation::DistanceSensor<S>,
        O: simulation::Observer<S>
    >(
        &mut self,
        spec: &ScenarioSpec,
        trajectory_sets: TrajectorySets<S>,
        controller: C,
        formation_generator: G,
        sensor: Se,
        mut observer: O,
    ) -> Result<()> {
        let num_sets = trajectory_sets.len();
        let set_num_width = num_sets.to_string().len();
        let mut rng = SmallRng::from_entropy();
        let num_robots = spec.robot.num_robots;
        let sensors = vec![sensor; num_robots];
        let controllers = vec![controller; num_robots];
        let mut total_path_error = 0.;

        for (set_num, (params, trajectories)) in trajectory_sets.into_iter().enumerate() {
            debug!("Processing trajectory set number {}", set_num);
            let mut file_description: DataFileDescription = Default::default();
            let file_name = format!("data{:0width$}.tfrecord", set_num, width = set_num_width);
            let file_path = self.data_dir.join(&file_name);
            file_description.file = file_path
                .strip_prefix(&self.working_dir)?
                .to_str()
                .ok_or(format_err!("weird characters in filename"))?
                .to_string();
            file_description.features = params;
            file_description.num_trajectories =
                spec.reference_trajectories.num_per_set * spec.robot.num_robots;

            let data_file = File::create(file_path)?;
            let mut writer = tf_record::ResultsWriter::<File, S>::from_writer(data_file)?;

            for trajectory in trajectories {
                let (formation_params, formation) =
                    formation_generator.generate(&mut rng, spec.robot.num_robots);
                for leader in 0..spec.robot.num_robots {
                    let result =
                        simulation::SimpleSimulation::new(
                            spec.robot.num_robots,
                            leader,
                            sensors.clone(),
                            controllers.clone(),
                            &formation,
                            &trajectory,
                            simulation::LeaderTrajectoryMode::Follow,
                        ).run(spec.length, spec.resolution, &mut observer);

                    let mut trajectory_params = formation_params.clone();
                    trajectory_params
                        .insert("leader".to_string(), ConstantParam::Int(leader as i64));
                    if let Some(error) = result.path_error() {
                        trajectory_params.insert("path_error".to_string(), ConstantParam::Float(error));
                        total_path_error += error;
                        trace!("Path error={}", error);
                    }

                    file_description
                        .per_trajectory_features
                        .push(trajectory_params);

                    writer.write_record(result, leader)?;
                }
            }

            self.description.files.push(file_description);
            writer.finish()?;
        }
        if total_path_error > 0. {
            let total_trajectories = (num_sets * spec.reference_trajectories.num_per_set * spec.robot.num_robots) as f64;
            let avg_path_error = total_path_error / total_trajectories;
            self.description.features.insert("avg_path_error".to_string(), ConstantParam::Float(avg_path_error));
        }
        let mut info_file = File::create(&self.info_file_path)?;
        write!(&mut info_file, "# datagen-info v2.1\n")?;
        serde_yaml::to_writer(&mut info_file, &self.description)?;

        info!("Finished writing file {:?}", self.info_file_path);
        Ok(())
    }

    fn get_prefix() -> String {
        time::strftime("dg-%Y_%m_%d-%H_%M_%S-", &time::now()).unwrap()
    }
}

impl ScenarioSpec {
    pub fn execute(&self) -> Result<PathBuf> {
        let mut executor = ScenarioExecutionContext::new();
        executor.execute(self)?;
        Ok(executor.into_info_file_path())
    }
}

struct TrajectoryGeneratorSimple1D {
    variability: f64,
    rsd: f64,
}

impl TrajectoryGeneratorSimple1D {
    pub fn from_params(params: &Params) -> Result<Self> {
        let variability = params
            .get("variability")
            .ok_or(format_err!("variability param not found"))?
            .as_f64()?;
        let rsd = params
            .get("rsd")
            .ok_or(format_err!("rsd param not found"))?
            .as_f64()?;
        Ok(TrajectoryGeneratorSimple1D { variability, rsd })
    }

    pub fn generate(
        &self,
        length: Seconds,
        resolution: Seconds,
        max_speed: MetresPerSecond,
    ) -> trajectory::NaiveTrajectory<Metres> {
        let points = trajectory::generate_1d_trajectory_points_simple(
            max_speed,
            length,
            self.variability,
            self.rsd,
        );
        trajectory::NaiveTrajectory::from_points(resolution, points)
    }
}

struct TrajectoryGeneratorSimple2D {
    variability: f64,
    rsd: f64,
    turnability: f64,
}

impl TrajectoryGeneratorSimple2D {
    pub fn from_params(params: &Params) -> Result<Self> {
        {
            let variability = params
                .get("variability")
                .ok_or(format_err!("variability param not found"))?
                .as_f64()?;
            let rsd = params
                .get("rsd")
                .ok_or(format_err!("rsd param not found"))?
                .as_f64()?;
            let turnability = params
                .get("turnability")
                .ok_or(format_err!("turnability param not found"))?
                .as_f64()?;
            Ok(TrajectoryGeneratorSimple2D {
                variability,
                rsd,
                turnability,
            })
        }
    }

    pub fn generate(
        &self,
        length: Seconds,
        resolution: Seconds,
        max_speed: MetresPerSecond,
    ) -> trajectory::NaiveTrajectory<Metres2D> {
        let points = trajectory::generate_2d_trajectory_points_simple(
            max_speed,
            length,
            self.variability,
            self.rsd,
            self.turnability,
        );
        trajectory::NaiveTrajectory::from_points(resolution, points)
    }
}

trait FormationGenerator<S: Vector> {
    type Result: simulation::Formation<S>;

    fn generate<R: Rng + ?Sized>(&self, rng: &mut R, num_robots: usize) -> (Params, Self::Result);
}

enum GenericFloatParam {
    Constant(f64),
    Uniform(Uniform<f64>),
    Normal(Normal),
}

impl GenericFloatParam {
    fn from_param(params: &ParamsSpec, key: &str) -> Result<Self> {
        match *params {
            ParamsSpec::Constant { ref values } => match *values
                .get(key)
                .ok_or(format_err!("required parameter not found"))?
                {
                    ConstantParam::Float(val) => Ok(GenericFloatParam::Constant(val)),
                    ConstantParam::Int(val) => Ok(GenericFloatParam::Constant(val as f64))
                },
            ParamsSpec::Random { ref values } => {
                match *values
                    .get(key)
                    .ok_or(format_err!("required parameter not found"))?
                    {
                        RandomParamSpec::Constant { value } => Ok(GenericFloatParam::Constant(value)),
                        RandomParamSpec::Uniform { range } => {
                            let (lower, upper) = range;
                            let dist = Uniform::new_inclusive(lower, upper);
                            Ok(GenericFloatParam::Uniform(dist))
                        }
                        RandomParamSpec::Normal { range } => {
                            let (lower, upper) = range;
                            let mean = (lower + upper) / 2.;
                            let sd = (mean - lower) / 2.58; // 99% of values in range
                            Ok(GenericFloatParam::Normal(Normal::new(mean, sd)))
                        }
                    }
            }
        }
    }

    fn sample<R: Rng + ?Sized>(&self, rng: &mut R) -> f64 {
        match *self {
            GenericFloatParam::Constant(val) => val,
            GenericFloatParam::Uniform(dist) => dist.sample(rng),
            GenericFloatParam::Normal(dist) => dist.sample(rng),
        }
    }
}

struct Simple1DFormationGenerator {
    distance: GenericFloatParam,
}

impl Simple1DFormationGenerator {
    fn new(params: &ParamsSpec) -> Result<Self> {
        Ok(Simple1DFormationGenerator {
            distance: GenericFloatParam::from_param(params, "distance")?,
        })
    }

    fn get_params(distance: f64) -> Params {
        let mut params: Params = HashMap::with_capacity(1);
        params.insert("distance".to_string(), ConstantParam::Float(distance));
        params
    }
}

impl FormationGenerator<Metres> for Simple1DFormationGenerator {
    type Result = simulation::SimpleFormation<Metres>;

    fn generate<R: Rng + ?Sized>(&self, rng: &mut R, num_robots: usize) -> (Params, Self::Result) {
        assert_eq!(num_robots, 2);
        let distance = self.distance.sample(rng);
        (
            Self::get_params(distance),
            simulation::SimpleFormation::new(2, 0., vec![distance, 0.]),
        )
    }
}

struct Dummy2DFormationGenerator;

impl FormationGenerator<Metres2D> for Dummy2DFormationGenerator {
    type Result = simulation::SimpleFormation<Metres2D>;

    fn generate<R: Rng + ?Sized>(&self, rng: &mut R, num_robots: usize) -> (Params, Self::Result) {
        panic!("2d formations not yet implemented")
    }
}
