use std::sync::Arc;
use std::hash::{Hash,Hasher};
use std::collections::HashSet;
use std::cmp::Ordering;
use std::marker::{Send,Sync};
use std::ops::{Deref, DerefMut};

pub type PointRef<T,U> = Arc<Point<T,U>>;
pub type PointHashSet<T,U> = HashSet<PointRef<T,U>>;
pub type PointXYZ<T> = Point<XYZ,T>;
pub type PointXYZRef<T> = Arc<PointXYZ<T>>;
pub type PointXYZIntensity = PointXYZ<Intensity>;
pub type PointXYZIntensityRef = Arc<PointXYZIntensity>;
pub type PointXYZRGB = PointXYZ<RGB>;
pub type PointXYZRGBRef = Arc<PointXYZRGB>;
pub type PointXYZPlain = PointXYZ<NoData>;
pub type PointXYZPlainRef = Arc<PointXYZPlain>;


// Used to access Point coordinates such as XYZ
pub trait PointMeasurable: Hash + Clone + Ord + HasDimensions + Send + Sync + std::fmt::Debug{}
// Used to access Data associated with the point such as RGB, Intensity or NoData
pub trait PointDataAccess: Hash + Clone + Ord + Send + Sync + std::fmt::Debug{}

#[derive(Debug,Clone,Copy)]
pub struct Point<T,U>
{
  point: T,
  data: Option<U>,
}

#[derive(Debug, Clone, Copy)]
pub struct XYZ{
  x: f32,
  y: f32,
  z: f32,
}

#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct RGB {
  r: u8,
  g: u8,
  b: u8,
}

#[derive(Debug, Clone, Copy)]
pub struct Intensity {
  i: f32,
}

// used when a plain point
#[derive(Debug, Clone, Copy, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct NoData {}

pub trait HasDimensions {
  fn new (x:f32, y:f32, z:f32 ) -> Self;
  fn dims() -> usize;
  fn array(&self) -> [f32;3];
  fn tuple(&self) -> (f32,f32,f32);
  fn dist(&self, other: &Self) -> f32;
}

impl HasDimensions for  XYZ {
  fn new (x:f32, y:f32, z:f32 ) -> Self {
    Self {x,y,z}
  }
  fn dims() -> usize {
    3 as usize
  }
  fn array(&self) -> [f32;3] {
    [self.x, self.y, self.z]
  }
  fn tuple(&self) -> (f32,f32,f32) {
    (self.x, self.y, self.z)
  }
  fn dist(&self, other: &Self) -> f32 {
    let dx = self.x - other.x;
    let dy: f32 = self.y - other.y;
    let dz: f32 = self.z - other.z;
    (dx*dx+dy*dy+dz*dz).sqrt()
  }
}
impl XYZ
where XYZ: HasDimensions {
  pub fn x(&self) -> f32 {
    self.x
  }
  pub fn y(&self) -> f32 {
    self.y
  }
  pub fn z(&self) -> f32 {
    self.z
  }
  pub fn xyz(&self) -> [f32;3]{
    [self.x, self.y, self.z]
  }
  pub fn tuple(&self) -> (f32,f32,f32){
    (self.x, self.y, self.z)
  }
}

impl PointMeasurable for XYZ {
}

impl Ord for XYZ {
  fn cmp(&self, other: &Self) -> Ordering {
    let t0 = self.tuple();
    let d0: i64 =((t0.0*t0.0 + t0.1*t0.1 + t0.2*t0.2).sqrt() *1000.0) as i64 ;
    let t1 = &other.tuple();
    let d1: i64 =((t1.0*t1.0 + t1.1*t1.1 + t1.2*t1.2).sqrt() *1000.0) as i64 ;
    d0.cmp(&d1)
  }
}

impl PartialOrd for XYZ {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}


// needed for f32 values
impl PartialEq for XYZ {
  fn eq(&self, other: &Self) -> bool {
    self.x == other.x && self.y == other.y && self.z == other.z
  }
}

impl Eq for XYZ{}

impl Hash for XYZ {
  fn hash<H: Hasher>(&self, state: &mut H) {
    let mul_factor = |x:f32| -> i64 {
      (x * 1000.0) as i64
    };
    mul_factor(self.x).hash(state);
    mul_factor(self.y).hash(state);
    mul_factor(self.z).hash(state);
  }
}

impl RGB {
  pub fn r(&self) -> u8 {
    self.r
  }
  pub fn g(&self) -> u8 {
    self.g
  }
  pub fn b(&self) -> u8 {
    self.b
  }
  pub fn rgb(&self) -> [u8;3]{
    [self.r, self.g, self.b]
  }
}

impl PointDataAccess for RGB {}

impl Intensity {
  pub fn i(&self) -> f32 {
    self.i
  }
  pub fn intensity(&self) -> i64 {
    (self.i * 10.0) as i64
  }
}

impl Ord for Intensity{
  fn cmp(&self, other: &Self) -> Ordering {
    self.intensity().cmp(&other.intensity())
  }
}

impl PartialOrd for Intensity{
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

impl PartialEq for Intensity {
  fn eq(&self, other: &Self) -> bool {
    self.i == other.i
  }
}

impl Eq for Intensity{}

impl Hash for Intensity {
  fn hash<H: Hasher>(&self, state: &mut H) {
    ((self.i * 10.0) as i64).hash(state);
  }
}

impl PointDataAccess for Intensity {}

impl PointDataAccess for NoData {}


pub trait HasPoint<T,U> {
  fn point(&self)->&T;
}


impl<T,U> HasPoint<T,U> for Point<T,U>
where T:PointMeasurable, U:PointDataAccess,
{
  fn point(&self) -> &T {
    &self.point
  }
}


pub trait HasData<T,U> {
  fn data(&self)->&Option<U>;
}

impl<T,U> HasData<T,U> for Point<T,U>
where T:PointMeasurable, U:PointDataAccess,
{
  fn data(&self) -> &Option<U> {
    &self.data
  }
}

impl<T,U> Hash for Point<T,U>
where T: Hash, U: Hash{
  fn hash<H:Hasher>(&self, state:&mut H){
    self.point.hash(state);
    self.data.hash(state);
  }
}

impl<T,U> Ord for Point<T,U>
where T:Ord, U: Ord {
  fn cmp(&self, other: &Self) -> Ordering {
    (&self.point, &self.data).cmp(&(&other.point,&other.data))
  }
}

impl<T,U> PartialOrd for Point<T,U>
where T:PartialOrd+Ord, U:PartialOrd+Ord {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.cmp(other))
  }
}

impl<T,U> PartialEq for Point<T,U>
where T:PartialEq, U:PartialEq {
  fn eq(&self, other: &Self) -> bool {
    (&self.point,&self.data) == (&other.point, &other.data)
  }
}

impl<T,U> Eq for Point<T,U>
where T:Eq, U:Eq
{}

pub trait HasNew<T,U> {
  fn new(point:T, data:Option<U>) -> Self;
}

impl <T,U> HasNew<T,U> for Point<T,U> {
    fn new(point:T, data:Option<U>) -> Self {
        Self {point, data}
    }

}

impl Point<XYZ,NoData> {
  pub fn x(&self) -> f32 {
    self.point.x
  }
  pub fn y(&self) -> f32 {
    self.point.y
  }
  pub fn z(&self) -> f32 {
    self.point.z
  }
  pub fn xyz(&self) -> [f32;3]{
    [self.point.x, self.point.y, self.point.z]
  }
  pub fn dist(&self, other:&Self) -> f32 {
    self.point.dist(&other.point)
  }
  pub fn new(
    x: f32,
    y: f32,
    z: f32,
  ) -> Self {
    let point = XYZ {x,y,z};
    Point::<XYZ,NoData> {point, data:None}
  }
}

impl Point<XYZ,Intensity> {
  pub fn x(&self) -> f32 {
    self.point.x
  }
  pub fn y(&self) -> f32 {
    self.point.y
  }
  pub fn z(&self) -> f32 {
    self.point.z
  }
  pub fn xyz(&self) -> [f32;3]{
    [self.point.x, self.point.y, self.point.z]
  }
  pub fn dist(&self, other:&Self) -> f32 {
    self.point.dist(&other.point)
  }

  pub fn new(
    x: f32,
    y: f32,
    z: f32,
    i: f32,
  ) -> Self {
    let point = XYZ {x,y,z};
    let data = Some(Intensity {i});
    Point::<XYZ,Intensity> {point, data}
  }
  pub fn i(&self) -> f32 {
    self.data.unwrap_or(Intensity {i:0.0}).i
  }
  pub fn intensity(&self) -> Intensity {
    self.data.unwrap_or(Intensity {i:0.0})
  }
}

impl Point<XYZ,RGB> {
  pub fn x(&self) -> f32 {
    self.point.x
  }
  pub fn y(&self) -> f32 {
    self.point.y
  }
  pub fn z(&self) -> f32 {
    self.point.z
  }
  pub fn xyz(&self) -> [f32;3]{
    [self.point.x, self.point.y, self.point.z]
  }
  pub fn new(
    x: f32,
    y: f32,
    z: f32,
    r: u8,
    g: u8,
    b: u8,
  ) -> Self {
    let point = XYZ {x,y,z};
    let data = Some(RGB {r,g,b});
    Point::<XYZ,RGB> {point, data}
  }
  pub fn r(&self) -> u8 {
    self.data.unwrap_or(RGB{r:0,g:0,b:0}).r
  }
  pub fn g(&self) -> u8 {
    self.data.unwrap_or(RGB{r:0,g:0,b:0}).g
  }
  pub fn b(&self) -> u8 {
    self.data.unwrap_or(RGB{r:0,g:0,b:0}).b
  }
  pub fn rgb(&self) -> RGB {
    self.data.unwrap_or(RGB{r:0,g:0,b:0})
  }

}
