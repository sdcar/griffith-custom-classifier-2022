extern crate sdcar_vision;
extern crate nalgebra as na;

use na::{Point2, Point3};

use sdcar_vision::stereo_calibration::StereoCalibration;
use sdcar_vision::lidar_to_camera::LiDAR2Camera;

fn main () {
  let calibration_path = "/home/nick/feature-extraction-and-map-poc5/sdcar_vision/calibration/calibration.yaml";

  let stereo_calibration = StereoCalibration::new(calibration_path);

  let lidar_to_camera = LiDAR2Camera::from_left_camera(stereo_calibration.clone());
  // println!("lidar_to_camera: {:#?}", lidar_to_camera);

  let p0 = Point3::new( 8.266552, 3.1580696, -0.69036144);

  println!("p0 {}", &p0);


  let p_2d =lidar_to_camera.project_lidar_to_image(p0).unwrap();

  println!("p_2d {}", p_2d);
}