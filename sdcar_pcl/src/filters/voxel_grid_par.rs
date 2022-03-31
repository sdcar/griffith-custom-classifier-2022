extern crate nalgebra;


// Note this approach is slower then the other
// the cost (or maybe its just blocking) of all the locking in DashMap isnt speeing things up

use crate::point_cloud::*;
use crate::point::*;

use rayon::prelude::*;
use std::hash::{Hash,Hasher};
use dashmap::DashMap;


use std::sync::Arc;
use std::cmp::Ordering;

use nalgebra::Vector3;

type MinVec = Vector3<f32>;
type MaxVec = Vector3<f32>;

pub fn get_min_max<PointXYZIntensity>(in_cloud: &PointCloudXYZIntensity) -> (MinVec, MaxVec) {
  let mm_pair = in_cloud.point_indices_ref()
    .par_iter()
    // .cloned()
    .map(|p| {
      let xyz=  p.point().tuple();
      (xyz, xyz)
    })
    .reduce(|| ((f32::MAX, f32::MAX, f32::MAX), (f32::MIN, f32::MIN, f32::MIN)),
      |a,b|{ 
          let minv = (a.0.0.min(b.0.0), a.0.1.min(b.0.1), a.0.2.min(b.0.2));
          let maxv = (a.1.0.max(b.1.0), a.1.1.max(b.1.1), a.1.2.max(b.1.2));
          (minv,maxv)
      }
    );

  (MinVec::new(mm_pair.0.0, mm_pair.0.1, mm_pair.0.2),
   MaxVec::new(mm_pair.1.0, mm_pair.1.1, mm_pair.1.2))
}

type PointVec = [f32;3];
type BoxPointMap = DashMap<i32,PointBox>;

#[derive(Debug, Clone, Copy)]
struct PointBox {
  box_idx: i32,
  centroid:  PointVec,
  intensity: f32,
  n_points: usize,
}

impl  PointBox {
    fn new(box_idx: i32) -> Self { Self { box_idx, centroid: [0.0,0.0,0.0], n_points:0, intensity: 0.0 } }
    fn update(&mut self, point: PointVec, intensity: f32) {
      self.centroid[0] += point[0];
      self.centroid[1] += point[1];
      self.centroid[2] += point[2];
      self.intensity += intensity;
      self.n_points += 1;
    }
    fn centroid (&self) -> PointVec {
      let c = [self.centroid[0],self.centroid[1],self.centroid[2]];
      let n = self.n_points as f32;
      [c[0]/n, c[1]/n,c[1]/n]
    }
    fn intensity (&self) -> f32 {
      self.intensity/ self.n_points as f32
    }
}

impl Ord for PointBox {
  fn cmp(&self, other: &Self) -> Ordering {
    self.box_idx.cmp(&other.box_idx)
  }
}

impl PartialOrd for PointBox {
  fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
    Some(self.box_idx.cmp(&other.box_idx))
  }
}

impl PartialEq for PointBox {
  fn eq(&self, other: &Self) -> bool {
    self.box_idx == other.box_idx
  }
}

impl Hash for PointBox {
  fn hash<H: Hasher>(&self, state: &mut H) {
    self.box_idx.hash(state);
  }
}

impl Eq for PointBox {}


// used this routine https://github.com/ucanbizon/downsampling-point-cloud/blob/master/downsample.cpp
pub fn filter_points(in_cloud: &PointCloudXYZIntensity, leaf_count: Vector3<usize>) -> PointCloudXYZIntensity {

  let mm_pair = get_min_max::<PointXYZIntensity>(in_cloud);
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

  let box_point_map = BoxPointMap::new();

  let box_point_map_update = |p: &PointXYZIntensityRef| {
    let ijk0 = ((p.point().x() * inv_leaf_size.x) - minb.x as f32).floor() as i32;
    let ijk1 = ((p.point().y() * inv_leaf_size.y) - minb.y as f32).floor() as i32;
    let ijk2 = ((p.point().z() * inv_leaf_size.z) - minb.z as f32).floor() as i32;
    let box_idx = ijk0 * divb_mul.x + ijk1 * divb_mul.y + ijk2 * divb_mul.z;

    let point = [p.point().x(), p.point().y(),p.point().z()];
    let intensity = p.i();

    match box_point_map.get_mut(&box_idx) {
      Some(mut x) => {x.update(point, intensity)},
      None => {
        let mut pb: PointBox = PointBox::new(box_idx);
        pb.update(point, intensity);
        box_point_map.insert(box_idx, pb);
      },

    };

  //   if !box_point_map.contains_key(&box_idx)
  //   {
  //     let mut pb = PointBox::new(box_idx);
  //     pb.update(point, intensity);
  //     box_point_map.insert(box_idx, PointBox::new(box_idx));
  //   } else {
  //     box_point_map.alter(&box_idx, |_,mut pb|{pb.alter(point, intensity)});
  //   }
  };

  point_indices_ref
  .par_iter()
  .for_each(|p| {box_point_map_update(p)});

  let out_cloud_indices_iter =  
  box_point_map
  .into_iter()
  .map(|(_box_idx, point_box)| {
    let centroid = point_box.centroid();
    let intensity=point_box.intensity();
 
    Arc::new(PointXYZIntensity::new(centroid[0], centroid[1],centroid[1],intensity))
  });

  let out_cloud_indices:PointXYZIntensityIndices = out_cloud_indices_iter
  .collect();

  PointCloudXYZIntensity::from(out_cloud_indices)
}