use base::*;
use std::ops::Index;

#[derive(Clone)]
pub struct NaiveTrajectory<S: Vector>(Seconds, Vec<S>);

impl<S: Vector> NaiveTrajectory<S> {
    pub fn time_step(&self) -> Seconds { self.0 }
    pub fn data(&self) -> &Vec<S> { &self.1 }
}

pub trait Trajectory<S: Vector> {
    fn pos_at_time(&self, time: Seconds) -> S;
    fn end_time(&self) -> Seconds;
}

pub trait UniformResolutionTrajectory<S: Vector>: Trajectory<S> + Index<usize, Output=S> {
    fn resolution(&self) -> Seconds;
}

impl<S: Vector> Trajectory<S> for NaiveTrajectory<S> {
    fn pos_at_time(&self, time: Seconds) -> S {
        if self.1.is_empty() { return S::zero(); }

        // Do linear interpolation between samples
        if time < 0. {
            return self.1[0];
        }
        let end_time = self.end_time();
        if time > end_time {
            return *self.1.last().unwrap();
        }

        let sample_coord = time / end_time;
        let sample_left = sample_coord.floor() as usize;
        let sample_right = sample_coord.ceil() as usize;
        let val_left = self.1[sample_left];
        let val_right = self.1[sample_right];
        let a = sample_coord.fract();

        val_left * a + val_right * (1. - a)
    }

    fn end_time(&self) -> Seconds { self.0 * ((self.1.len() - 1) as f64) }
}

impl<S: Vector> UniformResolutionTrajectory<S> for NaiveTrajectory<S> {
    fn resolution(&self) -> Seconds { self.time_step() }
}

impl<S: Vector> Index<usize> for NaiveTrajectory<S> {
    type Output = S;
    fn index(&self, index: usize) -> &S {
        &self.1[index]
    }
}