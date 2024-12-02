#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y
    }
    pub fn mag_sq(self) -> f64 {
        self.dot(self)
    }
    pub fn mag(self) -> f64 {
        self.mag_sq().sqrt()
    }
    pub fn normalised(self) -> Self {
        self / self.mag()
    }
}

use std::ops::*;

impl Add for Vec2 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self::Output {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self::Output {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Neg for Vec2 {
    type Output = Self;
    fn neg(self) -> Self::Output {
        Vec2::new(-self.x, -self.y)
    }
}
impl<T: Into<f64>> Mul<T> for Vec2 {
    type Output = Self;
    fn mul(self, rhs: T) -> Self::Output {
        let rhs = rhs.into();
        Vec2::new(self.x * rhs, self.y * rhs)
    }
}
impl<T: Into<f64>> Div<T> for Vec2 {
    type Output = Self;
    fn div(self, rhs: T) -> Self::Output {
        let rhs = rhs.into();
        Vec2::new(self.x / rhs, self.y / rhs)
    }
}
impl From<[f64; 2]> for Vec2 {
    fn from(v: [f64; 2]) -> Self {
        Self::new(v[0], v[1])
    }
}
impl From<&[f64; 2]> for Vec2 {
    fn from(v: &[f64; 2]) -> Self {
        (*v).into()
    }
}

impl From<(f64, f64)> for Vec2 {
    fn from(v: (f64, f64)) -> Self {
        Self::new(v.0, v.1)
    }
}
impl From<&(f64, f64)> for Vec2 {
    fn from(v: &(f64, f64)) -> Self {
        Self::new(v.0, v.1)
    }
}

impl std::ops::Index<usize> for Vec2 {
    type Output = f64;
    fn index(&self, idx: usize) -> &Self::Output {
        match idx {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Index out of bounds on vec2!"),
        }
    }
}

impl std::ops::IndexMut<usize> for Vec2 {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        match idx {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Index out of bounds on vec2!"),
        }
    }
}
