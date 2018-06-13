use base::*;
use failure::Fail;
use num::Zero;
use std::f64;
use std::f64::consts::PI;
use std::fmt;
use std::fmt::Display;
use std::fmt::Error as FmtError;
use std::fmt::Formatter;
use std::iter;
use std::iter::Peekable;
use std::ops::Add;
use std::slice;

pub mod bindings;

pub use self::bindings::DubinsPathType;
use rand::distributions::{Distribution, Uniform};
use rand::Rng;
use std::cell::Cell;

#[derive(Clone)]
struct PeekableMap<I, F> {
    iter: I,
    fun: F,
}

impl<I, F> PeekableMap<I, F> {
    pub fn new(iter: I, fun: F) -> Self {
        PeekableMap { iter, fun }
    }
}

impl<I: fmt::Debug, F> fmt::Debug for PeekableMap<I, F> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.debug_struct("PeekableMap")
            .field("iter", &self.iter)
            .finish()
    }
}

impl<B, I, F> Iterator for PeekableMap<Peekable<I>, F>
where
    I: Iterator,
    F: FnMut(<Peekable<I> as Iterator>::Item, Option<&<Peekable<I> as Iterator>::Item>) -> B,
{
    type Item = B;

    #[inline]
    fn next(&mut self) -> Option<B> {
        self.iter
            .next()
            .map(|item| (self.fun)(item, self.iter.peek()))
    }

    #[inline]
    fn size_hint(&self) -> (usize, Option<usize>) {
        self.iter.size_hint()
    }
}

trait WithPeekableMap: Iterator {
    fn map_with_peek<F, B>(self, f: F) -> PeekableMap<Self, F>
    where
        F: FnMut(<Self as Iterator>::Item, Option<&<Self as Iterator>::Item>) -> B,
        Self: Sized;
}

impl<I> WithPeekableMap for Peekable<I>
where
    I: Iterator,
{
    fn map_with_peek<F, B>(self, f: F) -> PeekableMap<Self, F>
    where
        F: FnMut(<Self as Iterator>::Item, Option<&<Self as Iterator>::Item>) -> B,
        Self: Sized,
    {
        PeekableMap::new(self, f)
    }
}

#[derive(Debug, Fail)]
pub enum DubinsError {
    ColocatedConfigurations,
    PathParametrisationError,
    BadRho,
    NoPath,
    Unknown(u32),
}

impl Display for DubinsError {
    fn fmt(&self, f: &mut Formatter) -> Result<(), FmtError> {
        match *self {
            DubinsError::ColocatedConfigurations => write!(f, "Colocated configurations"),
            DubinsError::PathParametrisationError => write!(f, "Path parametrisation error"),
            DubinsError::BadRho => write!(f, "Rho value was invalid"),
            DubinsError::NoPath => write!(f, "No path connecting the configurations"),
            DubinsError::Unknown(code) => write!(f, "Unknown error code: {}", code),
        }
    }
}

impl DubinsError {
    pub fn from_c_errcode(c_err: u32) -> Result<(), DubinsError> {
        match c_err {
            bindings::EDUBOK => Ok(()),
            bindings::EDUBCOCONFIGS => Err(DubinsError::ColocatedConfigurations),
            bindings::EDUBPARAM => Err(DubinsError::PathParametrisationError),
            bindings::EDUBBADRHO => Err(DubinsError::BadRho),
            bindings::EDUBNOPATH => Err(DubinsError::NoPath),
            code => Err(DubinsError::Unknown(code)),
        }
    }
}

type Configuration = [f64; 3];

trait ConvertibleToConfiguration {
    fn to_configuration(&self) -> Configuration;
}

impl ConvertibleToConfiguration for OrientedPosition2D {
    fn to_configuration(&self) -> [f64; 3] {
        [self.position.x, self.position.y, self.rotation]
    }
}

trait ConvertibleToOrientedPosition2D {
    fn to_oriented_position(&self) -> OrientedPosition2D;
}

impl ConvertibleToOrientedPosition2D for Configuration {
    fn to_oriented_position(&self) -> OrientedPosition2D {
        OrientedPosition2D::new(self[0], self[1], self[2])
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DubinsPath {
    inner: Cell<bindings::DubinsPath>,
    end: OrientedPosition2D,
}

unsafe extern "C" fn dubins_sampling_callback(
    q: *mut f64,
    t: f64,
    user_data: *mut ::std::os::raw::c_void,
) -> ::std::os::raw::c_int {
    let data: &mut Vec<(f64, OrientedPosition2D)> =
        &mut *(user_data as *mut Vec<(f64, OrientedPosition2D)>);
    let config = slice::from_raw_parts(q, 3);
    let position = OrientedPosition2D::new(config[0], config[1], config[2]);
    data.push((t, position));
    return 0;
}

impl DubinsPath {
    pub fn with_type(
        start: OrientedPosition2D,
        end: OrientedPosition2D,
        turning_radius: f64,
        shape: DubinsPathType,
    ) -> Result<Self, DubinsError> {
        let mut path: bindings::DubinsPath = Default::default();
        let mut start_ary = start.to_configuration();
        let mut end_ary = end.to_configuration();
        unsafe {
            let res = bindings::dubins_path(
                &mut path as *mut _,
                start_ary.as_mut_ptr(),
                end_ary.as_mut_ptr(),
                turning_radius,
                shape,
            ) as u32;
            DubinsError::from_c_errcode(res)?;
        }
        Ok(DubinsPath {
            inner: Cell::new(path),
            end,
        })
    }

    pub fn new_shortest(
        start: OrientedPosition2D,
        end: OrientedPosition2D,
        turning_radius: f64,
    ) -> Result<Self, DubinsError> {
        let mut path: bindings::DubinsPath = Default::default();
        let mut start_ary = start.to_configuration();
        let mut end_ary = end.to_configuration();
        unsafe {
            let res = bindings::dubins_shortest_path(
                &mut path as *mut _,
                start_ary.as_mut_ptr(),
                end_ary.as_mut_ptr(),
                turning_radius,
            ) as u32;
            DubinsError::from_c_errcode(res)?;
        }
        Ok(DubinsPath {
            inner: Cell::new(path),
            end,
        })
    }

    pub fn length(&self) -> f64 {
        unsafe { bindings::dubins_path_length(self.inner.as_ptr()) }
    }

    /// Segment 0-2
    pub fn segment_length(&self, segment: u32) -> f64 {
        unsafe { bindings::dubins_segment_length(self.inner.as_ptr(), segment as _) }
    }

    pub fn segment_length_normalized(&self, segment: u32) -> f64 {
        unsafe { bindings::dubins_segment_length_normalized(self.inner.as_ptr(), segment as _) }
    }

    pub fn path_type(&self) -> DubinsPathType {
        unsafe { bindings::dubins_path_type(self.inner.as_ptr()) }
    }

    pub fn endpoint(&self) -> Result<OrientedPosition2D, DubinsError> {
        let mut endpoint: Configuration = Default::default();
        unsafe {
            let res =
                bindings::dubins_path_endpoint(self.inner.as_ptr(), endpoint.as_mut_ptr()) as u32;
            DubinsError::from_c_errcode(res)?;
        }
        Ok(endpoint.to_oriented_position())
    }

    pub fn nominal_end(&self) -> OrientedPosition2D {
        self.end
    }

    pub fn subpath(&self, length: f64) -> Result<DubinsPath, DubinsError> {
        let mut path: bindings::DubinsPath = Default::default();
        unsafe {
            let res =
                bindings::dubins_extract_subpath(self.inner.as_ptr(), length, &mut path as *mut _);
            DubinsError::from_c_errcode(res as _)?;
        }
        let mut new_path = DubinsPath {
            inner: Cell::new(path),
            end: Default::default(),
        };
        new_path.end = new_path.endpoint()?;
        Ok(new_path)
    }

    pub fn sample(&self, position: f64) -> Result<OrientedPosition2D, DubinsError> {
        let mut q: Configuration = Default::default();
        unsafe {
            let res = bindings::dubins_path_sample(self.inner.as_ptr(), position, q.as_mut_ptr());
            DubinsError::from_c_errcode(res as _)?;
        }
        Ok(q.to_oriented_position())
    }

    pub fn to_uniform_data(
        &self,
        resolution: f64,
    ) -> Result<Vec<(f64, OrientedPosition2D)>, DubinsError> {
        let mut results: Vec<(f64, OrientedPosition2D)> =
            Vec::with_capacity((self.length() / resolution).ceil() as _);
        unsafe {
            let res = bindings::dubins_path_sample_many(
                self.inner.as_ptr(),
                resolution,
                Some(dubins_sampling_callback),
                &mut results as *mut _ as _,
            );
            DubinsError::from_c_errcode(res as _)?;
        }
        Ok(results)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MultiDubinsPath {
    subpaths: Vec<DubinsPath>,
    speed: MetresPerSecond,
    total_length: Metres,
    path_lengths: Vec<Metres>,
}

impl MultiDubinsPath {
    pub fn to_dynamic_trajectory(
        &self,
        resolution: f64,
    ) -> Vec<(
        Seconds,
        (Metres2D, Radians),
        (MetresPerSecond, RadiansPerSecond),
    )> {
        let sampling_resolution = resolution * self.speed; // sample every n metres to achieve sampling every n seconds

        self.subpaths
            .iter()
            .enumerate()
            .flat_map(|(index, subpath)| {
                // todo dirty hack here, would actually want to bail on first error
                let data = subpath
                    .to_uniform_data(sampling_resolution)
                    .expect("Could not sample subpath");
                let x_offset = self.path_lengths.iter().take(index).fold(0., Add::add);
                data.into_iter()
                    .map(move |(x, position)| ((x + x_offset) / self.speed, position))
            })
            .chain({
                let final_time = self.total_length / self.speed;
                let last_position = self.subpaths.last().unwrap().nominal_end();
                iter::once((final_time, last_position))
            })
            .peekable()
            .map_with_peek(|this_config, next_config| {
                let (now_time, now_position) = this_config;
                let now_movement = if let Some(&(next_time, next_position)) = next_config {
                    let speed = (next_position.position - now_position.position).length()
                        / (next_time - now_time);
                    let omega =
                        (next_position.rotation - now_position.rotation) / (next_time - now_time);
                    (speed, omega)
                } else {
                    (0., 0.)
                };

                (
                    now_time,
                    (now_position.position, now_position.rotation),
                    now_movement,
                )
            })
            .collect::<Vec<_>>()
    }

    pub fn generate<R: Rng + ?Sized>(
        turning_radius: Metres,
        speed: MetresPerSecond,
        min_length: Seconds,
        rng: &mut R,
        origin: OrientedPosition2D,
        range: Metres,
    ) -> Result<Self, DubinsError> {
        let min_distance = min_length * speed;
        let mut subpaths: Vec<DubinsPath> = Vec::with_capacity(1);
        let mut current_distance = 0.;
        let mut end_config = origin;
        while current_distance < min_distance {
            let start_config = end_config;
            end_config = Self::random_config(rng, range, origin.position);
            let subpath = DubinsPath::new_shortest(start_config, end_config, turning_radius)?;
            current_distance += subpath.length();
            subpaths.push(subpath);
        }
        let path_lengths: Vec<Metres> = subpaths.iter().map(|subpath| subpath.length()).collect();
        let total_length = path_lengths.iter().fold(0., Add::add);

        Ok(MultiDubinsPath {
            subpaths,
            speed,
            total_length,
            path_lengths,
        })
    }

    pub fn length(&self) -> Seconds {
        self.total_length / self.speed
    }

    pub fn sample(&self, t: Seconds) -> Result<(Metres2D, Radians), DubinsError> {
        let mut sampling_distance = t * self.speed;
        if sampling_distance > self.total_length {
            return Err(DubinsError::PathParametrisationError);
        }

        let mut sample: (Metres2D, Radians) = Default::default();
        for path in self.subpaths.iter() {
            if path.length() < sampling_distance {
                sampling_distance -= path.length();
                continue;
            }

            let o_pos = path.sample(sampling_distance)?;
            sample = (o_pos.position, o_pos.rotation);
            break;
        }

        Ok(sample)
    }

    pub fn endpoint(&self) -> (Metres2D, Radians) {
        match self.subpaths.last() {
            None => (Metres2D::zero(), PI / 2.),
            Some(path) => {
                let endpoint = path.nominal_end();
                (endpoint.position, endpoint.rotation)
            }
        }
    }

    fn random_config<R: Rng + ?Sized>(
        rng: &mut R,
        range: Metres,
        origin: Metres2D,
    ) -> OrientedPosition2D {
        let pos_dist = Uniform::new_inclusive(-range, range);
        let x = pos_dist.sample(rng);
        let y = pos_dist.sample(rng);
        let rot_dist = Uniform::new(0., 2. * PI);
        let theta = rot_dist.sample(rng);
        let mut pos = OrientedPosition2D::new(x, y, theta);
        pos.position += origin;
        pos
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use base::*;
    use rand::thread_rng;

    #[test]
    fn basic_dubins() {
        let start = OrientedPosition2D::new(0., 0., 0.);
        let end = OrientedPosition2D::new(10., 0., 0.);
        let path = DubinsPath::new_shortest(start, end, 1.).expect("Could not calculate path");
        let data = path.to_uniform_data(0.5).expect("Could not sample path");
        let shape = path.path_type();
        let lens = [
            path.segment_length(0),
            path.segment_length(1),
            path.segment_length(2),
        ];
        println!("{:?}(L{:?}): {:?}({:?})", data, path.length(), shape, lens);
    }

    #[test]
    fn multi_dubins() {
        let mut rng = thread_rng();
        let origin = OrientedPosition2D::new(0., 0., PI / 2.);
        let multi = MultiDubinsPath::generate(1., 2., 15., &mut rng, origin, 10.)
            .expect("could not generate");
        let data = multi.to_dynamic_trajectory(0.5);
        println!("{:?}", data);
    }
}
