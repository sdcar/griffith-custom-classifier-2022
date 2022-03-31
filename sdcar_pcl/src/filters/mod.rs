pub mod voxel_grid;
pub mod voxel_grid_par;
pub mod crop_box;

use crate::point::*;
use crate::point_cloud::*;

use rayon::prelude::*;
use std::collections::HashSet;
use std::sync::Arc;

use nalgebra::Vector3;

pub type MinVec = Vector3<f32>;
pub type MaxVec = Vector3<f32>;


// pub fn outlier_ref_filter(
//     cloud: &PointCloudXYZIntensity,
//     inliers_ref: PointXYZIntensityIndicesRef,
// ) -> PointXYZIntensityIndicesRef {
//     let point_indices_ref = cloud.point_indices_ref();

//     let inliers_copy: HashSet<PointXYZIntensityRef> =
//         inliers_ref.par_iter()
//         .cloned()
//         .map(|p| p).collect();

//     let out_filter = point_indices_ref
//         .par_iter()
//         .cloned()
//         .filter(|p| !inliers_copy.contains(p));
//     let outliers: PointXYZIntensityIndices = out_filter.collect();
//     Arc::new(outliers)
// }


pub fn outlier_filter<T,U> 
(
    cloud: &PointCloud<Point<T,U>>,
    inliers_ref: PointIndicesRef<Point<T,U>>,
) -> PointIndicesRef<Point<T,U>> 
where T:PointMeasurable, 
      U:PointDataAccess

{
    let point_indices_ref = cloud.point_indices_ref();

    let inliers_copy: HashSet<PointRef<T,U>> =
        inliers_ref.par_iter()
        .cloned()
        .map(|p| p).collect();

    let out_filter = point_indices_ref
        .par_iter()
        .cloned()
        .filter(|p| !inliers_copy.contains(p));
    let outliers: PointIndices<Point<T,U>> = out_filter.collect();
    Arc::new(outliers)

}


pub fn get_min_max<T,U> (in_cloud: &PointCloud<Point<T,U>>) -> (MinVec, MaxVec) 
where T:PointMeasurable, 
      U:PointDataAccess
{
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

pub fn get_pir_min_max<T,U> (point_indices_ref: &PointIndicesRef<Point<T,U>>) -> (MinVec, MaxVec) 
where T:PointMeasurable, 
      U:PointDataAccess
{
  let mm_pair = point_indices_ref
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