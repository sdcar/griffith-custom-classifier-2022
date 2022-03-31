extern crate nalgebra as na;
extern crate opencv;
extern crate sdcar_pcl;

use na::{Matrix3, Matrix3x4, Matrix4, Point2, Point3};

use crate::lidar_to_camera::LiDAR2Camera;
use crate::bounding_box::{ImageBoundingBoxXYZIntensity,ImageBoundingBoxesXYZIntensityRef};
use opencv::core::Point2i;

use sdcar_pcl::prelude::*;
use rayon::prelude::*;

#[derive(Debug, Clone)]
pub struct ClusterLiDARWithTmageBB {
  lidar_to_camera: LiDAR2Camera
}

impl ClusterLiDARWithTmageBB {
    pub fn new(lidar_to_camera: LiDAR2Camera) -> Self { Self { lidar_to_camera } }

    pub fn cluster(&self, point_cloud_ref: PointCloudXYZIntensityRef, image_bbs_ref: ImageBoundingBoxesXYZIntensityRef, shrink_factor: f32) {

      let image_bb_num = image_bbs_ref.read().unwrap().len();
      // for image_bb in image_bbs_ref.write().unwrap().iter_mut() {
      //   image_bb.lidar_points = None;
      // }
      let cluster_lp_to_bb = |i| {
            let lp = point_cloud_ref.point(i).clone();
            let (x, y, z) = lp.clone().point().tuple();
            let x = x as f64;
            let y = y as f64;
            let z = z as f64;

            let p_2d = self.lidar_to_camera.project_lidar_to_image(na::Point3::new(x.into(),y.into(),z.into())).unwrap();

            for n in 0..image_bb_num {
              if image_bbs_ref.read().unwrap().get(n).unwrap().contains_with_shrinkage(Point2i::new(p_2d.x as i32, p_2d.y as i32), shrink_factor) {
                image_bbs_ref.write().unwrap().get_mut(n).unwrap().append_lidar_point(lp.clone());
                break; // only append each lidar point to one image bounding box
              }
            }
          };

      (0..point_cloud_ref.num_points())
      .into_par_iter()
      // .for_each(|i| cluster_lp_to_bb(i));
      .for_each( cluster_lp_to_bb);
    }
}