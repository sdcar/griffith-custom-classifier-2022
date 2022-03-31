extern crate sdcar_pcl;
extern crate opencv;

use opencv::core::{
    Mat
};

use std::collections::HashMap;
use sdcar_pcl::prelude::*;
use crate::bounding_box::*;
#[derive(Debug, Clone)]
pub struct ImageFrame<T,U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
  pub camera_img: Mat,

  pub lidar_points: Option<PointIndicesRef<Point<T,U>>>,
  pub lidar_bounding_boxes: Option<BoundingBoxesRef<T,U>>,

  pub image_bounding_boxes: Option<ImageBoundingBoxes<T,U>>, // ROI detected in 2D image coordinates
  pub bb_matches: Option<HashMap<u32,u32>>, // matches between previous and current frame
}

impl<T,U> ImageFrame <T,U>
where
    T: PointMeasurable,
    U: PointDataAccess,

{
  pub fn new(camera_img: Mat, lidar_points: Option<PointIndicesRef<Point<T,U>>>, lidar_bounding_boxes: Option<BoundingBoxesRef<T,U>>, image_bounding_boxes: Option<ImageBoundingBoxes<T,U>>, bb_matches: Option<HashMap<u32,u32>>) -> Self  {
    Self {camera_img, lidar_points, lidar_bounding_boxes, image_bounding_boxes, bb_matches}
  }
}