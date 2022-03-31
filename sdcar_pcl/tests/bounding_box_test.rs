extern crate sdcar_pcl;

use sdcar_pcl::point::*;
use sdcar_pcl::point_cloud::*;
use sdcar_pcl::filters::*;
use sdcar_pcl::bounding_box::*;

use std::sync::Arc;
mod setup_data;

#[test]
fn bounding_box_creation_test () {
  let point_indices_ref = setup_data::build_point_indices();

  let num_points = point_indices_ref.len();
  assert!(num_points==4, "{} points loaded", num_points);

  let cloud = PointCloudXYZIntensity::from(point_indices_ref);

  let (min_vec, max_vec) = get_min_max(&cloud);

  println!("{:?} {:?}", min_vec, max_vec);

  let point_min = Point::<XYZ,Intensity>::new(min_vec.x, min_vec.y, min_vec.z, 0.0);
  let point_max = Point::<XYZ,Intensity>::new(max_vec.x, max_vec.y, max_vec.z, 0.0);

  let bb = BoundingBox::<XYZ,Intensity>::new(None, None, None, point_min, point_max);
  println!("{:?} dist {}", bb, bb.dist());

  let point_inside = PointXYZIntensityRef::new(PointXYZIntensity::new(-6.0, 7.0,0.0,0.0));
  assert!(bb.bounds(&point_inside), "{:?} point not bounded by bb {:?}", point_inside, bb);
  
  let point_outside = PointXYZIntensityRef::new(PointXYZIntensity::new(-6.4, 7.0,0.0,0.0));
  assert!(!bb.bounds(&point_outside), "{:?} point outside bounded by bb {:?}", point_inside, bb);
}

#[test]
fn bounding_box_lines_test() {
  
  let point_indices_ref = setup_data::build_point_indices();
  let cloud = PointCloudXYZIntensity::from(point_indices_ref);
  let bb = BoundingBox::<XYZ,Intensity>::from(Arc::new(cloud));

  println!("{:?}",bb);

  let lines = bb.frame_lines();
  assert!(lines.len() == 12, "Expected 12 lines found {}", lines.len());
  for line in lines.clone() {
    println!("{:?}", line)
  }
}