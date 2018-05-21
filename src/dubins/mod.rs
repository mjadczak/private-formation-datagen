use std::fmt::Display;
use std::fmt::Formatter;
use std::fmt::Error as FmtError;
use failure::Fail;
use base::*;
use std::slice;
use std::f64;

pub mod bindings;

pub use self::bindings::DubinsPathType;
use std::cell::Cell;

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

pub struct DubinsPath {
    inner: Cell<bindings::DubinsPath>,
    end: OrientedPosition2D,
}

unsafe extern "C" fn dubins_sampling_callback(q: *mut f64, t: f64, user_data: *mut ::std::os::raw::c_void) -> ::std::os::raw::c_int {
    let data: &mut Vec<(f64, OrientedPosition2D)> = &mut *(user_data as *mut Vec<(f64, OrientedPosition2D)>);
    let config = slice::from_raw_parts(q, 3);
    let position = OrientedPosition2D::new(config[0], config[1], config[2]);
    data.push((t, position));
    return 0;
}

impl DubinsPath {
    pub fn with_type(start: OrientedPosition2D, end: OrientedPosition2D, turning_radius: f64, shape: DubinsPathType) -> Result<Self, DubinsError> {
        let mut path: bindings::DubinsPath = Default::default();
        let mut start_ary = start.to_configuration();
        let mut end_ary = end.to_configuration();
        unsafe {
            let res = bindings::dubins_path(&mut path as *mut _, start_ary.as_mut_ptr(), end_ary.as_mut_ptr(), turning_radius, shape) as u32;
            DubinsError::from_c_errcode(res)?;
        }
        Ok(DubinsPath { inner: Cell::new(path), end })
    }

    pub fn new_shortest(start: OrientedPosition2D, end: OrientedPosition2D, turning_radius: f64) -> Result<Self, DubinsError> {
        let mut path: bindings::DubinsPath = Default::default();
        let mut start_ary = start.to_configuration();
        let mut end_ary = end.to_configuration();
        unsafe {
            let res = bindings::dubins_shortest_path(&mut path as *mut _, start_ary.as_mut_ptr(), end_ary.as_mut_ptr(), turning_radius) as u32;
            DubinsError::from_c_errcode(res)?;
        }
        Ok(DubinsPath { inner: Cell::new(path), end })
    }

    pub fn length(&self) -> f64 {
        unsafe {
            bindings::dubins_path_length(self.inner.as_ptr())
        }
    }

    /// Segment 0-2
    pub fn segment_length(&self, segment: u32) -> f64 {
        unsafe {
            bindings::dubins_segment_length(self.inner.as_ptr(), segment as _)
        }
    }

    pub fn segment_length_normalized(&self, segment: u32) -> f64 {
        unsafe {
            bindings::dubins_segment_length_normalized(self.inner.as_ptr(), segment as _)
        }
    }

    pub fn path_type(&self) -> DubinsPathType {
        unsafe {
            bindings::dubins_path_type(self.inner.as_ptr())
        }
    }

    pub fn endpoint(&self) -> Result<OrientedPosition2D, DubinsError> {
        let mut endpoint: Configuration = Default::default();
        unsafe {
            let res = bindings::dubins_path_endpoint(self.inner.as_ptr(), endpoint.as_mut_ptr()) as u32;
            DubinsError::from_c_errcode(res)?;
        }
        Ok(endpoint.to_oriented_position())
    }

    pub fn subpath(&self, length: f64) -> Result<DubinsPath, DubinsError> {
        let mut path: bindings::DubinsPath = Default::default();
        unsafe {
            let res = bindings::dubins_extract_subpath(self.inner.as_ptr(), length, &mut path as *mut _);
            DubinsError::from_c_errcode(res as _)?;
        }
        let mut new_path = DubinsPath { inner: Cell::new(path), end: Default::default() };
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

    pub fn to_uniform_data(&self, resolution: f64) -> Result<Vec<(f64, OrientedPosition2D)>, DubinsError> {
        let mut results: Vec<(f64, OrientedPosition2D)> = Vec::with_capacity((self.length() / resolution).ceil() as _);
        unsafe {
            let res = bindings::dubins_path_sample_many(self.inner.as_ptr(), resolution, Some(dubins_sampling_callback), &mut results as *mut _ as _);
            DubinsError::from_c_errcode(res as _)?;
        }
        results.push((self.length(), self.end));
        Ok(results)
    }
}

#[cfg(test)]
mod tests {
    use base::*;
    use super::*;

    #[test]
    fn basic_dubins() {
        let start = OrientedPosition2D::new(0., 0., 0.);
        let end = OrientedPosition2D::new(10., 0., 0.);
        let path = DubinsPath::new_shortest(start, end, 1.).expect("Could not calculate path");
        let data = path.to_uniform_data(0.5).expect("Could not sample path");
        let shape = path.path_type();
        let lens = [path.segment_length(0), path.segment_length(1), path.segment_length(2)];
        println!("{:?}(L{:?}): {:?}({:?})", data, path.length(), shape, lens);
    }
}