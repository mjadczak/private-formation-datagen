use num::Zero;
use std::ops::*;

pub trait Vector:
    PartialEq
    + Add<Output = Self>
    + Sub<Output = Self>
    + AddAssign
    + SubAssign
    + MulAssign<f64>
    + DivAssign<f64>
    + Mul<f64, Output = Self>
    + Div<f64, Output = Self>
    + PartialOrd
    + Sized
    + Copy
    + Zero
{
    fn length(&self) -> f64;

    fn repr_length() -> usize;

    fn repr(&self) -> Vec<f64>;
}

pub type Seconds = f64;
pub type Metres = f64;
pub type MetresPerSecond = f64;
pub type Radians = f64;

impl Vector for f64 {
    fn length(&self) -> f64 {
        *self
    }

    fn repr_length() -> usize {
        1
    }

    fn repr(&self) -> Vec<f64> {
        vec![*self]
    }
}

#[derive(PartialEq, PartialOrd, Copy, Clone, Debug)]
pub struct Metres2D {
    pub x: f64,
    pub y: f64,
}

pub type Metres2DPerSecond = Metres2D;

impl Add for Metres2D {
    type Output = Metres2D;

    fn add(self, rhs: Metres2D) -> Metres2D {
        Metres2D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl Sub for Metres2D {
    type Output = Metres2D;

    fn sub(self, rhs: Metres2D) -> Metres2D {
        Metres2D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Mul<f64> for Metres2D {
    type Output = Metres2D;

    fn mul(self, rhs: f64) -> Metres2D {
        Metres2D {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Div<f64> for Metres2D {
    type Output = Metres2D;

    fn div(self, rhs: f64) -> Metres2D {
        Metres2D {
            x: self.x / rhs,
            y: self.y / rhs,
        }
    }
}

impl Zero for Metres2D {
    fn zero() -> Self {
        Metres2D { x: 0., y: 0. }
    }

    fn is_zero(&self) -> bool {
        self.x == 0. && self.y == 0.
    }
}

impl AddAssign for Metres2D {
    fn add_assign(&mut self, rhs: Metres2D) {
        *self = *self + rhs;
    }
}

impl SubAssign for Metres2D {
    fn sub_assign(&mut self, rhs: Metres2D) {
        *self = *self - rhs;
    }
}

impl MulAssign<f64> for Metres2D {
    fn mul_assign(&mut self, rhs: f64) {
        *self = *self * rhs;
    }
}

impl DivAssign<f64> for Metres2D {
    fn div_assign(&mut self, rhs: f64) {
        *self = *self / rhs;
    }
}

impl Vector for Metres2D {
    fn length(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    fn repr_length() -> usize {
        2
    }

    fn repr(&self) -> Vec<f64> {
        vec![self.x, self.y]
    }
}

pub struct PolarMetres2D {
    pub theta: f64,
    pub r: f64,
}

impl PolarMetres2D {
    pub fn to_cartesian(&self) -> Metres2D {
        Metres2D {
            x: self.r * self.theta.cos(),
            y: self.r * self.theta.sin(),
        }
    }
}

impl Metres2D {
    pub fn to_polar(&self) -> PolarMetres2D {
        PolarMetres2D {
            theta: self.y.atan2(self.x),
            r: self.length(),
        }
    }
}
