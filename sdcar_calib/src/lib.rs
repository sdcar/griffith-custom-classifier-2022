pub mod ego;
pub mod lidar;
extern crate sdcar_pcl;
extern crate opencv;

use opencv::core::{
    Mat
};

use std::collections::HashMap;
use sdcar_pcl::prelude::*;
use sdcar_vision::bounding_box::*;
#[derive(Debug, Clone)]
pub struct CalibFrame<T,U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
  pub camera_img: Mat,

  pub lidar_points: Option<PointCloudRef<Point<T,U>>>,
  pub lidar_bounding_boxes: Option<BoundingBoxesRef<T,U>>,
  pub target_lidar_points: Option<PointIndicesRef<Point<T,U>>>,
  pub target_lidar_bb: Option<BoundingBoxRef<T,U>>,

  pub image_bounding_boxes: Option<ImageBoundingBoxesRef<T,U>>, // ROI detected in 2D image coordinates
  pub target_image_bb: Option<ImageBoundingBox<T,U>>,
}

impl<T,U> CalibFrame<T, U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    pub fn new(camera_img: Mat, lidar_points: Option<PointCloudRef<Point<T,U>>>, lidar_bounding_boxes: Option<BoundingBoxesRef<T,U>>, target_lidar_points: Option<PointIndicesRef<Point<T,U>>>, target_lidar_bb: Option<BoundingBoxRef<T,U>>, image_bounding_boxes: Option<ImageBoundingBoxesRef<T,U>>, target_image_bb: Option<ImageBoundingBox<T,U>>) -> Self {
        Self { camera_img, lidar_points, lidar_bounding_boxes, target_lidar_points, target_lidar_bb, image_bounding_boxes, target_image_bb}
    }
}



#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
