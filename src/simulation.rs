use std::ops::*;

pub trait Vector: PartialEq + Add + Sub + Mul + PartialOrd + Sized {}

impl Vector for f64 {}

pub trait Controller<SpaceEl: Vector>: Clone {
    type Params: Clone + Default;

    fn params(&self) -> Self::Params;
    fn target(&self) -> SpaceEl;
    fn set_target(&mut self, target: SpaceEl);

    /// Takes the current measured distance to the target, and returns the new velocity vector
    fn take_step(&mut self, distance: SpaceEl) -> SpaceEl;
}

/// A simplified view of a formation for N robots. Some robot is defined to be the leader and
/// all others attempt to keep their distance from it as defined by the formation.
/// The origin specifies their initial positions in a simulation.
pub trait Formation<SpaceEl: Vector> {
    fn num_robots(&self) -> usize;
    fn origin(&self) -> SpaceEl;
    fn positions(&self) -> &[SpaceEl];
}

/// A result of a simulation for multiple robots, with a fixed-size time step
pub trait SimulationResult<SpaceEl: Vector> {
    fn time_step(&self) -> f64;
    fn num_robots(&self) -> usize;
    fn num_steps(&self) -> usize;

    /// First dimension is robot number, second dimension is timestep
    fn results(&self) -> &[&[SpaceEl]];
}

pub trait Simulation<SpaceEl: Vector, Result: SimulationResult<SpaceEl>> {
    fn run(self, total_time: f64, time_step: f64) -> Result;
}


