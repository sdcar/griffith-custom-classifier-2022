extern crate sdcar_vision;
extern crate nalgebra as na;

use na::{Vector5, Matrix3, Matrix4, Matrix3x4};

use sdcar_vision::stereo_calibration::StereoCalibration;

fn main () {
  let calibration_path = "/home/nick/feature-extraction-and-map-poc5/sdcar_vision/calibration/calibration.yaml";

  let stereo_calib = StereoCalibration::new(calibration_path);

  println!("k1 {:?}", stereo_calib.k1());
  println!("d1 {:?}", stereo_calib.d1());
  println!("k2 {:?}", stereo_calib.k2());
  println!("d2 {:?}", stereo_calib.d2());
  println!("r {:?}", stereo_calib.r());
  println!("t {:?}", stereo_calib.t());
  println!("e {:?}", stereo_calib.e());
  println!("f {:?}", stereo_calib.f());
  println!("r1 {:?}", stereo_calib.r1());
  println!("r2 {:?}", stereo_calib.r2());
  println!("p1 {:?}", stereo_calib.p1());
  println!("p2 {:?}", stereo_calib.p2());
  println!("q {:?}", stereo_calib.q());
}
