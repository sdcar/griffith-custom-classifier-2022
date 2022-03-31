
pub mod extract_clusters;
pub mod sac;

use crate::point_cloud::*;
use crate::point::*;
use crate::filters::*;

use sac::ransac;

use std::sync::Arc;

pub fn segment_planes_ransac(
  cloud: &PointCloudXYZIntensity,
  max_iterations: usize,
  distance_to_l: f32,
) -> (PointCloudXYZIntensityRef, PointCloudXYZIntensityRef) {
  let inliers_ref = ransac(cloud, max_iterations, distance_to_l);

  let outlier_ref = outlier_filter::<XYZ,Intensity>(cloud, inliers_ref.clone());

  let inlier_cloud_ref = Arc::new(PointCloudXYZIntensity::from(inliers_ref));
  let outlier_cloud_ref = Arc::new(PointCloudXYZIntensity::from(outlier_ref));

  (inlier_cloud_ref, outlier_cloud_ref)
}