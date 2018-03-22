use std::ops::*;
use num::{Zero, One};

pub trait Vector: PartialEq + Add + Sub + Mul<f64, Output=Self> + Div<f64, Output=Self> + PartialOrd + Sized + Copy + Zero + One {}

pub type Seconds = f64;
pub type Metres = f64;
pub type MetresPerSecond = f64;

impl Vector for f64 {}