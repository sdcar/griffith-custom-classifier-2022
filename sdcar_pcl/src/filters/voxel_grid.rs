extern crate nalgebra;

use crate::point_cloud::*;
use crate::point::*;
use crate::filters::get_min_max;

use rayon::prelude::*;

use std::sync::Arc;
use std::cmp::Ordering;

use nalgebra::Vector3;

#[derive(Debug, Clone, Eq, PartialEq)]
struct PointBox<T> {
  box_idx: i32,
  point_ref: Arc<T>,
}

impl PartialOrd for PointBox<PointXYZIntensity> {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    // Some(self.cmp(other))
    self.box_idx.partial_cmp(&other.box_idx)
  }
}


// used this routine https://github.com/ucanbizon/downsampling-point-cloud/blob/master/downsample.cpp
pub fn filter_points(in_cloud: &PointCloudXYZIntensity, leaf_count: Vector3<usize>) -> PointCloudXYZIntensity {
  let mm_pair = get_min_max::<XYZ,Intensity>(in_cloud);
  let minp = mm_pair.0;
  let maxp = mm_pair.1;

  
  // let leaf_count = Vector3::<usize>::new(0,0,0);

  let leaf_size = Vector3::<f32>::new(
    (maxp.x - minp.x)/leaf_count.x as f32,
    (maxp.y - minp.y)/leaf_count.y as f32,
    (maxp.z - minp.z)/leaf_count.z as f32);
  let inv_leaf_size = Vector3::<f32>::new(1.0f32 /leaf_size.x, 1.0f32/leaf_size.y, 1.0f32/leaf_size.z);

  // compute the min and max bounding box
  let minb = Vector3::<i32>::new (
    (minp.x * inv_leaf_size.x).floor() as i32, 
    (minp.y * inv_leaf_size.y).floor() as i32, 
    (minp.z * inv_leaf_size.z).floor() as i32) ;

  let maxb = Vector3::<i32>::new (
    (maxp.x * inv_leaf_size.x).ceil() as i32, 
    (maxp.y * inv_leaf_size.y).ceil() as i32, 
    (maxp.z * inv_leaf_size.z).ceil() as i32) ;

  let divb = Vector3::<i32>::new (
    maxb.x - minb.x + 1,
    maxb.y - minb.y + 1,
    maxb.z - minb.z + 1 );

  let divb_mul = Vector3::<i32>::new (1, divb.x, divb.x * divb.y);

  let point_indices_ref = in_cloud.point_indices_ref();
  let bb_indices_par_iter = Box::new(point_indices_ref
     .par_iter()
     .cloned()
     .map(|p| -> PointBox<PointXYZIntensity>{
      let ijk0 = ((p.point().x() * inv_leaf_size.x) - minb.x as f32).floor() as i32;
      let ijk1 = ((p.point().y() * inv_leaf_size.y) - minb.y as f32).floor() as i32;
      let ijk2 = ((p.point().z() * inv_leaf_size.z) - minb.z as f32).floor() as i32;
      let box_idx = ijk0 * divb_mul.x + ijk1 * divb_mul.y + ijk2 * divb_mul.z;
      PointBox::<PointXYZIntensity> {point_ref: p, box_idx} 
      }));

  let mut bb_indices:Vec<PointBox<PointXYZIntensity>> = bb_indices_par_iter.collect();
  bb_indices.par_sort_by(|a: &PointBox<_>, b: &PointBox<_>| b.partial_cmp(a).unwrap());

  let mut out_cloud_indices = PointXYZIntensityIndices::new();

  let mut cp:usize = 0;
  let mut centroid = Vector3::<f32>::zeros();
  let num_points = point_indices_ref.len();
  while cp < num_points {
    let c = bb_indices[cp].clone();
    let cpoint = c.point_ref.point();
    centroid.x = cpoint.x();
    centroid.y = cpoint.y();
    centroid.z = cpoint.z();

    let mut i = cp + 1;
    let mut intensity = c.point_ref.data().unwrap();
    while i < num_points && c.box_idx == bb_indices[i].box_idx {
      let point =  bb_indices[i].point_ref.point();
      intensity = bb_indices[i].point_ref.data().unwrap(); 
      // centroid += Vector3::<f32>::new(point.x(), point.y(), point.z());
      centroid.x += point.x();
      centroid.y += point.y();
      centroid.z += point.z();
      i+=1;
    } 

    centroid.x /= (i -cp) as f32;
    centroid.y /= (i -cp) as f32;
    centroid.z /= (i -cp) as f32;

    let p = Arc::new(PointXYZIntensity::new(centroid.x, centroid.y, centroid.z, intensity.i()));
    out_cloud_indices.push(p);
    cp = i;
  }
  PointCloudXYZIntensity::from(out_cloud_indices)
}