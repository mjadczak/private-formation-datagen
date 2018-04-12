use base::*;
use std::ops::Index;
use rand::{SmallRng, SeedableRng, thread_rng, Rng};
use rand::distributions::{Normal, Distribution, Uniform};
use std::f64::consts::PI;
use num::Zero;

#[derive(Clone, Debug)]
pub struct NaiveTrajectory<S: Vector>(Seconds, Vec<S>);

impl<S: Vector> NaiveTrajectory<S> {
    pub fn time_step(&self) -> Seconds { self.0 }
    pub fn data(&self) -> &Vec<S> { &self.1 }

    /// Creates a piecewise linear uniform NaiveTrajectory from a set of control points.
    /// The points should be at integer multiples of the resolution, and must be ordered in increasing order.
    /// The first point should have time = 0
    pub fn from_points<I>(resolution: Seconds, points: I) -> NaiveTrajectory<S>
        where I: IntoIterator<Item=(Seconds, S)>
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

/// A simple, non-configurable trajectory generation algorithm.
/// `variability` controls how many segments the trajectory has – the segments generated have a mean length of `min_length / variability`.
/// `rsd` controls how much the segment length varies.
///
/// The algorithm hardcodes a probability of 1/4 stationary, 1/4 full-speed with 3/4 chance forward, and 1/2 randomly-chosen speed.
/// This could be made configurable in the future.
///
/// The trajectory always starts at the origin.
pub fn generate_1d_trajectory_points_simple(max_speed: MetresPerSecond, min_length: Seconds, variability: f64, rsd: f64) -> Vec<(Seconds, Metres)> {
    let mut rng = SmallRng::from_rng(thread_rng()).unwrap();
    let segment_length_dist = {
        let mean = min_length / variability;
        let sd = mean * rsd;
        Normal::new(mean, sd)
    };
    let generator = SpeedGenerator::new(max_speed, 0.2, 0.7, 0.85);
    let mut cur_time = 0.;
    let mut cur_pos = 0.;
    let mut points = vec![(cur_time, cur_pos)];

    while cur_time < min_length {
        let seg_length = segment_length_dist.sample(&mut rng);
        if seg_length <= 0. { continue }
        cur_time += seg_length;
        let velocity = generator.gen_speed(&mut rng);
        cur_pos += velocity * seg_length;
        points.push((cur_time, cur_pos));
    }

    points
}


/// A version of `generate_1d_trajectory_points_simple` adapted for 2D.
pub fn generate_2d_trajectory_points_simple(max_speed: MetresPerSecond, min_length: Seconds, variability: f64, rsd: f64, turnability: f64) -> Vec<(Seconds, Metres2D)> {
    let mut rng = SmallRng::from_rng(thread_rng()).unwrap();
    let segment_length_dist = {
        let mean = min_length / variability;
        let sd = mean * rsd;
        Normal::new(mean, sd)
    };
    let heading_dist = Normal::new(0., turnability);
    let speed_generator = SpeedGenerator::new(max_speed, 0.2, 0.7, 0.85);
    let mut cur_time = 0.;
    let mut cur_pos = Metres2D::zero();
    let mut cur_heading = {
        let sample: f64 = rng.sample(Uniform);
        (sample * 2. * PI) - PI
    };
    let mut points = vec![(cur_time, cur_pos)];

    while cur_time < min_length {
        let seg_length = segment_length_dist.sample(&mut rng);
        if seg_length <= 0. { continue }
        cur_time += seg_length;
        let speed = speed_generator.gen_speed(&mut rng);
        cur_heading += heading_dist.sample(&mut rng);
        let velocity = PolarMetres2D {
            r: speed,
            theta: cur_heading
        }.to_cartesian();
        cur_pos += velocity * seg_length;
        points.push((cur_time, cur_pos));
    }

    points
}

struct SpeedGenerator {
    max_speed: MetresPerSecond,
    thresh_fullspeed: f64,
    thresh_randomspeed: f64,
    thresh_forward: f64,
    speed_dist: Normal,
}

impl SpeedGenerator {
    fn new(max_speed: MetresPerSecond, thresh_fullspeed: f64, thresh_randomspeed: f64, thresh_forward: f64) -> Self {
        let speed_sd = max_speed / 1.96;
        // 95% of returned values will lie between -max_speed and max_speed
        let speed_dist = Normal::new(0., speed_sd);
        SpeedGenerator {
            max_speed,
            thresh_fullspeed,
            thresh_randomspeed,
            thresh_forward,
            speed_dist,
        }
    }

    #[inline]
    fn clamp_speed(&self, val: f64) -> f64 {
        val.max(-self.max_speed).min(self.max_speed)
    }

    fn gen_speed<R: Rng + ?Sized>(&self, rng: &mut R) -> MetresPerSecond {
        let sample: f64 = rng.gen();
        // TODO make thresh names less confusing
        if sample < self.thresh_fullspeed {
            return 0.;
        }
        if sample < self.thresh_randomspeed {
            let is_forward = rng.gen::<f64>() < self.thresh_forward;
            return if is_forward { self.max_speed } else { -self.max_speed };
        }
        return self.clamp_speed(self.speed_dist.sample(rng));
    }
}