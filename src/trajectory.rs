use base::*;

#[derive(Clone)]
pub struct NaiveTrajectory<SpaceEl: Vector>(Seconds, Vec<SpaceEl>);

impl<S: Vector> NaiveTrajectory<S> {
    pub fn time_step(&self) -> Seconds { self.0 }
    pub fn data(&self) -> &Vec<S> { &self.1 }
}