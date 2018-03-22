use std::ops::*;

pub trait Vector: PartialEq + Add + Sub + Mul + PartialOrd + Sized + Copy {}

pub type Seconds = f64;
pub type Metres = f64;
pub type MetresPerSecond = f64;

impl Vector for f64 {}