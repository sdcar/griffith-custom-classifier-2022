extern crate opencv;

use opencv::core::{Point3d, Point2i, Rect2i,Mat, Vector, no_array};

use opencv::Result;

use opencv::prelude::*;
use std::sync::Arc;

pub type PinholeCameraRef = Arc<PinholeCamera>;

#[derive(Debug,Clone)]
pub struct CameraInfo{
  pub name: String,
  pub camera_matrix: Mat, // Camera Matrix A
  pub dist_coeffs: Vector<f64>,

}

impl CameraInfo {
    pub fn new(name: String, camera_matrix: Mat, dist_coeffs: Vector<f64>) -> Self {
      Self { name, camera_matrix, dist_coeffs }
    }

    pub fn new_brio() -> Self{
      let name = String::from("brio");
      let camera_matrix = Mat::from_slice_2d(&[
        &[755.925188, 0., 632.358702],
        &[0., 756.065280, 346.628380],
        &[0.,0.,1.],
      ]).unwrap();
      let mut dist_coeffs = Vector::<f64>::new();
      dist_coeffs.push(0.15477853459080293);
      dist_coeffs.push(-0.2540365747115118);
      dist_coeffs.push(-0.0022176383627433476);
      dist_coeffs.push(0.0008191771550834852);
      dist_coeffs.push(0.);


      Self {name, camera_matrix, dist_coeffs}
    }
    pub fn new_daheng_left() -> Self{
      let name = String::from("daheng_left");
      let camera_matrix = Mat::from_slice_2d(&[
        &[1.8019272106229043e+03, 0., 1.0147315383836959e+03],
        &[0., 1.8004645266867349e+03, 7.5597151164616434e+02],
        &[0.,0.,1.],
      ]).unwrap();
      let mut dist_coeffs = Vector::<f64>::new();
      dist_coeffs.push( -1.3119230098169488e-01);
      dist_coeffs.push( 9.1329954362277616e-02);
      dist_coeffs.push(-1.3612752872624691e-03);
      dist_coeffs.push(7.8415640338919232e-04);
      dist_coeffs.push(-1.8662647933248181e-02);


      Self {name, camera_matrix, dist_coeffs}
    }
    pub fn fx(&self)-> f64 {
      *self.camera_matrix.at_2d::<f64>(0, 0).unwrap()
    }
    pub fn fy(&self)-> f64 {
      *self.camera_matrix.at_2d::<f64>(1, 1).unwrap()
    }
    pub fn cx(&self)-> f64 {
      *self.camera_matrix.at_2d::<f64>(0, 2).unwrap()
    }
    pub fn cy(&self)-> f64 {
      *self.camera_matrix.at_2d::<f64>(1, 2).unwrap()
    }
    pub fn distortion_coefficients(&self) -> Vector<f64> {
      self.dist_coeffs.clone()
    }
}


#[derive(Debug,Clone,Copy)]
pub enum DistortionState
{
  None,
  Calibrated,
  Unknown,
}

#[derive(Debug,Clone)]
pub struct PinholeCamera
{
  pub distortion_state: DistortionState,
  pub camera_info: CameraInfo,
  pub rotation_v: Vector<f64>, // Rotation Vector
  pub translation_v: Vector<f64>, // Translation Vector
}


impl PinholeCamera {
  pub fn new(camera_info:CameraInfo, rotation_v: Vector<f64>, translation_v: Vector<f64>) -> Self {
    let ci = camera_info.clone();
    let distortion_state = if ci.distortion_coefficients().len() == 0 {
      DistortionState::Unknown
    } else {
      DistortionState::Calibrated
    };
    Self {distortion_state, camera_info, rotation_v, translation_v}
  }

  pub fn project_point(&self, point: Point3d) -> Result<(Point2i)> {
    let mut image_points = Vector::<Point2i>::new();
    let mut object_points = Vector::<Point3d>::new();
    object_points.push(point);

    opencv::calib3d::project_points(&object_points, &self.rotation_v, &self.translation_v, &self.camera_info.camera_matrix, &self.camera_info.dist_coeffs, &mut image_points, &mut no_array(), 0.0)?;
    Ok(image_points.get(0)?)
  }

  pub fn  update_rt(&mut self, rvecs: Vector<f64>, tvecs: Vector<f64>){
    self.rotation_v = rvecs;
    self.translation_v = tvecs;
  }
}