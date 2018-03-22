use base::*;
use std::ops::Index;

#[derive(Clone, Debug)]
pub struct NaiveTrajectory<S: Vector>(Seconds, Vec<S>);

impl<S: Vector> NaiveTrajectory<S> {
    pub fn time_step(&self) -> Seconds { self.0 }
    pub fn data(&self) -> &Vec<S> { &self.1 }

    /// Creates a piecewise linear uniform NaiveTrajectory from a set of control points.
    /// The points should be at integer multiples of the resolution, and must be ordered in increasing order.
    /// The first point should have time = 0
    pub fn from_points<I>(resolution: Seconds, points: I) -> NaiveTrajectory<S>
        where I: IntoIterator<Item = (Seconds, S)>
    {
        let mut data: Vec<S> = Vec::new();
        let mut pk = points.into_iter().peekable();
        if let None = pk.peek() {
            panic!("must pass in some points");
        }

        // Don't use a float as an iterator so that we don't accumulate errors
        let mut cur_index: u64 = 0;

        loop {
            let (left_time, left_pos) = pk.next().unwrap();
            let (right_time, right_pos) = match pk.peek() {
                None => break,
                Some(&(t, p)) => (t, p)
            };

            let time_span = right_time - left_time;

            loop {
                let cur_time = cur_index as f64 * resolution;
                if cur_time > right_time { break; }

                let a = (cur_time - left_time) / time_span;
                let pos = left_pos * (1. - a) + right_pos * a;
                data.push(pos);

                cur_index += 1;
            }
        }

        NaiveTrajectory(resolution, data)
    }
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
        let time_left = sample_left as f64 * self.0;
        let time_right = sample_right as f64 * self.0;
        let time_span = time_right - time_left;
        let val_left = self.1[sample_left];
        let val_right = self.1[sample_right];
        let a = (time - time_left) / time_span;

        val_left * (1. - a) + val_right * a
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