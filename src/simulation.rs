use pid_control::{PIDController as PIDControllerImpl, Controller as PIDControllerT};
use base::*;
use trajectory::NaiveTrajectory;
use rand::{NewRng, SmallRng, Rng};
use rand::distributions::StandardNormal;

pub trait Controller<S: Vector>: Clone {
    type Params: Clone + Default;

    fn params(&self) -> Self::Params;
    fn target(&self) -> S;
    fn set_target(&mut self, target: S);

    /// Takes the current measured distance to the target, and returns the new velocity vector
    fn take_step(&mut self, distance: S, time_step: Seconds) -> S;
}

/// A simplified view of a formation for N robots. Some robot is defined to be the leader and
/// all others attempt to keep their distance from it as defined by the formation.
/// The origin specifies their initial positions in a simulation.
pub trait Formation<S: Vector>: Clone {
    fn num_robots(&self) -> usize;
    fn origin(&self) -> S;
    fn positions(&self) -> &[S];
}

/// An observed result of a simulation for multiple robots, with a fixed-size time step
pub trait SimulationResult<S: Vector> {
    fn time_step(&self) -> Seconds;
    fn num_robots(&self) -> usize;
    fn num_steps(&self) -> usize;

    /// First dimension is robot number, second dimension is step
    fn into_data(self) -> Vec<Vec<S>>;
}

pub trait Simulation<S: Vector> {
    type Result: SimulationResult<S>;

    fn run(self, total_time: Seconds, time_step: Seconds, observer: &mut Observer<S>) -> Self::Result;
}

#[derive(Debug, Clone)]
pub struct SimpleFormation<S: Vector>(usize, S, Vec<S>);

impl<S: Vector> SimpleFormation<S> {
    pub fn new(num_robots: usize, origin: S, positions: Vec<S>) -> Self {
        assert_eq!(positions.len(), num_robots);
        SimpleFormation(num_robots, origin, positions)
    }
}

impl<S: Vector> Formation<S> for SimpleFormation<S> {
    fn num_robots(&self) -> usize { self.0 }
    fn origin(&self) -> S { self.1 }
    fn positions(&self) -> &[S] { &self.2[..] }
}

#[derive(Debug, Clone, Copy)]
pub struct PControllerParams {
    pub p_gain: f64,
    pub vel_limits: (MetresPerSecond, MetresPerSecond),
}

impl Default for PControllerParams {
    fn default() -> Self {
        PControllerParams {
            p_gain: -1.5,
            vel_limits: (-0.5, 0.5),
        }
    }
}

#[derive(Debug, Clone)]
pub struct PController {
    controller: PIDControllerImpl,
    params: PControllerParams,
}

impl PController {
    pub fn new(params: PControllerParams) -> Self {
        let mut controller = PIDControllerImpl::new(params.p_gain, 0., 0.);
        controller.set_limits(params.vel_limits.0, params.vel_limits.1);
        PController {
            params,
            controller,
        }
    }

    pub fn reset(&mut self) {
        self.controller.reset()
    }
}

impl Controller<f64> for PController {
    type Params = PControllerParams;

    fn params(&self) -> PControllerParams {
        self.params
    }

    fn target(&self) -> Metres {
        self.controller.target()
    }

    fn set_target(&mut self, target: Metres) {
        self.controller.set_target(target)
    }

    fn take_step(&mut self, distance: Metres, time_step: Seconds) -> MetresPerSecond {
        self.controller.update(distance, time_step)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PIDControllerParams {
    pub p_gain: f64,
    pub i_gain: f64,
    pub d_gain: f64,
    pub vel_limits: (MetresPerSecond, MetresPerSecond),
}

impl Default for PIDControllerParams {
    fn default() -> Self {
        PIDControllerParams {
            p_gain: -1.5,
            i_gain: -0.5,
            d_gain: -0.5,
            vel_limits: (-0.5, 0.5),
        }
    }
}

#[derive(Debug, Clone)]
pub struct PIDController {
    controller: PIDControllerImpl,
    params: PIDControllerParams,
}

impl PIDController {
    pub fn new(params: PIDControllerParams) -> Self {
        let mut controller = PIDControllerImpl::new(params.p_gain, params.i_gain, params.d_gain);
        controller.set_limits(params.vel_limits.0, params.vel_limits.1);
        PIDController {
            params,
            controller,
        }
    }

    pub fn reset(&mut self) {
        self.controller.reset()
    }
}

impl Controller<f64> for PIDController {
    type Params = PIDControllerParams;

    fn params(&self) -> PIDControllerParams {
        self.params
    }

    fn target(&self) -> Metres {
        self.controller.target()
    }

    fn set_target(&mut self, target: Metres) {
        self.controller.set_target(target)
    }

    fn take_step(&mut self, distance: Metres, time_step: Seconds) -> MetresPerSecond {
        self.controller.update(distance, time_step)
    }
}

pub struct SimpleSimulationResult<S: Vector>(Seconds, Vec<Vec<S>>);

impl<S: Vector> SimulationResult<S> for SimpleSimulationResult<S> {
    fn time_step(&self) -> Seconds {
        self.0
    }

    fn num_robots(&self) -> usize {
        self.1.len()
    }

    fn num_steps(&self) -> usize {
        if self.1.len() > 0 {
            self.1[0].len()
        } else {
            0
        }
    }

    fn into_data(self) -> Vec<Vec<S>> {
        self.1
    }
}

#[derive(PartialEq)]
pub enum LeaderFollowMode {
    /// Simply use the reference trajectory as the exact trajectory of the leader
    DefinedTrajectory,

    /// Use the reference trajectory as something for the leader to follow
    FollowTrajectory,
}

pub trait DistanceSensor {
    fn sense(&mut self, true_d: f64) -> f64;
}

pub struct SharpIrSensor {
    rng: SmallRng
}

impl SharpIrSensor {
    pub fn new() -> SharpIrSensor {
        let rng = SmallRng::new();
        SharpIrSensor { rng }
    }
}

impl Default for SharpIrSensor {
    fn default() -> Self {
        SharpIrSensor::new()
    }
}

const SHARP_IR_EQN: [f64; 3] = [-0.020077009250469, 0.120573832696841, 0.003295559781587];
impl DistanceSensor for SharpIrSensor {
    fn sense(&mut self, true_d: f64) -> f64 {
        let d2 = true_d * true_d;
        let d3 = true_d * d2;
        let sd = d3 * SHARP_IR_EQN[0] + d2 * SHARP_IR_EQN[1] + true_d * SHARP_IR_EQN[0];
        let error = self.rng.sample(StandardNormal) * sd;
        true_d + error
    }
}

pub trait Observer<S: Vector> {
    fn observe(&mut self, true_pos: S) -> S;
}

pub struct SimpleObserver {
    sd: f64,
    rng: SmallRng
}

impl SimpleObserver {
    pub fn new(sd: f64) -> Self {
        SimpleObserver {
            sd,
            rng: SmallRng::new()
        }
    }
}

impl Default for SimpleObserver {
    fn default() -> Self {
        Self::new(1.)
    }
}

impl Observer<Metres> for SimpleObserver {
    fn observe(&mut self, true_pos: f64) -> f64 {
        let error = self.rng.sample(StandardNormal) * self.sd;
        true_pos + error
    }
}

pub struct PerfectObserver {}

impl<S: Vector> Observer<S> for PerfectObserver {
    fn observe(&mut self, true_pos: S) -> S {
        true_pos
    }
}

/// Simple simulation in Metres and Seconds which uses the same controller for every robot
pub struct SimpleSimulation<C: Controller<Metres>> {
    num_robots: usize,
    leader_id: usize,
    controllers: Vec<C>,
    results: Vec<Vec<Metres>>,
    current_pos: Vec<Metres>,
    current_vel: Vec<MetresPerSecond>,
    trajectory: NaiveTrajectory<Metres>,
    trajectory_origin: Metres,
    follow_mode: LeaderFollowMode
}

impl<C> SimpleSimulation<C>
    where C: Controller<Metres>
{
    pub fn new<CI: IntoIterator<Item=C>, F: Formation<Metres>>(num_robots: usize, leader_id: usize,
                                                           controllers: CI, formation: &F,
                                                           trajectory: &NaiveTrajectory<Metres>, follow_mode: LeaderFollowMode)
                                                           -> Self {
        let mut controllers: Vec<C> = controllers.into_iter().collect();
        assert!(leader_id < num_robots);
        assert!(num_robots > 0);
        assert_eq!(controllers.len(), num_robots);
        assert_eq!(formation.num_robots(), num_robots);
        assert_eq!(formation.positions().len(), num_robots);
        let current_pos: Vec<Metres> = formation.positions().iter().map(|pos| pos - formation.origin()).collect();

        // Set targets for controllers
        let leader_pos = current_pos[leader_id];
        for (i, c) in controllers.iter_mut().enumerate() {
            c.set_target(leader_pos - current_pos[i]);
        }

        SimpleSimulation {
            num_robots,
            leader_id,
            controllers,
            results: vec![Vec::new(); num_robots],
            current_pos,
            current_vel: vec![0.; num_robots],
            trajectory: trajectory.clone(),
            trajectory_origin: leader_pos,
            follow_mode
        }
    }
}

impl<C> Simulation<Metres> for SimpleSimulation<C>
    where C: Controller<Metres>
{
    type Result = SimpleSimulationResult<Metres>;

    /// Runs the entire simulation synchronously.
    /// Runs for an integer number of `time_step`s, potentially stopping past `total_time` if they are not multiples
    fn run(mut self, total_time: Seconds, time_step: Seconds, observer: &mut Observer<Metres>) -> SimpleSimulationResult<Metres> {
        assert_eq!(self.trajectory.time_step(), time_step);
        let num_steps: usize = (total_time / time_step).ceil() as _;
        assert!(self.trajectory.data().len() >= num_steps);

        // Allocate space
        for res_vec in self.results.iter_mut() {
            res_vec.reserve(num_steps);
        }

        // TODO: make the sensors configurable (possibly per-robot?)
        let mut sensors: Vec<SharpIrSensor> = Vec::with_capacity(self.num_robots);
        for _ in 0..self.num_robots {
            sensors.push(SharpIrSensor::new())
        }

        for step in 0..num_steps {
            let reference_pos = self.trajectory.data()[step] + self.trajectory_origin;

            // Handle case where the leader exactly takes the reference trajectory
            if self.follow_mode == LeaderFollowMode::DefinedTrajectory {
                self.current_pos[self.leader_id] = reference_pos;
            }

            // First, update the current position of each robot based on the velocity since last time step
            // Also, record the current position at this step into the results
            // Vel is kept at 0 in the case of DefinedTrajectory, so this is fine
            for ((mut pos, vel), mut result) in self.current_pos.iter_mut().zip(self.current_vel.iter()).zip(self.results.iter_mut()) {
                *pos += vel;
                result.push(observer.observe(*pos));
            }

            // Now run the controllers for each robot, obtaining the new velocity for the next time slice
            let leader_pos = self.current_pos[self.leader_id];
            for (id, (mut controller, mut velocity)) in self.controllers.iter_mut().zip(self.current_vel.iter_mut()).enumerate() {
                let target_pos =
                    if id == self.leader_id {
                        match self.follow_mode {
                            LeaderFollowMode::DefinedTrajectory => continue,
                            LeaderFollowMode::FollowTrajectory => reference_pos
                        }
                    } else {
                        leader_pos
                    };
                let target_distance = target_pos - self.current_pos[id];
                let sensed_distance = sensors[id].sense(target_distance);
                *velocity = controller.take_step(sensed_distance, time_step);
            }
        }

        SimpleSimulationResult(time_step, self.results)
    }
}