extern crate sdcar_pcl;
extern crate opencv;

use opencv::core::{
   Point2i, Rect2i, Vector
};
use sdcar_pcl::prelude::*;
use std::sync::{Arc,RwLock};

pub type ImageBoundingBoxXYZIntensity = ImageBoundingBox<XYZ, Intensity>;
pub type ImageBoundingBoxesXYZIntensity = Vec<ImageBoundingBoxXYZIntensity>;
pub type ImageBoundingBoxes<T,U> = Vec<ImageBoundingBox<T,U>>;
pub type ImageBoundingBoxesRef<T,U> = Arc<RwLock<ImageBoundingBoxes<T,U>>>;
pub type ImageBoundingBoxesXYZIntensityRef = ImageBoundingBoxesRef<XYZ, Intensity>;


#[derive(Debug,Clone)]
pub struct ImageBoundingBox<T,U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
  pub box_id: u32,
  pub track_id: Option<u32>,

  pub roi: Rect2i, // 2d region of interest in image coordinates

  pub class_id: u32, // id corresponding to YOLO
  pub class: String, // coco class name
  pub confidence: f32, // classification trust

  pub  lidar_points: Option<PointIndices<Point<T,U>>>,

  // TODO if decide to use keypoints add keypoints and keypoint matches

}

impl<T,U> ImageBoundingBox <T,U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
  pub fn new(box_id: u32, roi: Rect2i, class_id: u32, class:String, confidence: f32) -> Self {
    Self {box_id, track_id:None, roi, class_id, class, confidence, lidar_points:None}
  }

  pub fn contains(&self, pt: Point2i) -> bool {
    self.roi.contains(pt)
  }

  pub fn contains_with_shrinkage(&self, pt: Point2i, shrink_factor: f32) -> bool{
    let mut smaller_roi = Rect2i::default();
    smaller_roi.x = self.roi.x + (shrink_factor * self.roi.width as f32 / 2.0) as i32;
    smaller_roi.y = self.roi.y + (shrink_factor * self.roi.height  as f32 / 2.0) as i32;
    smaller_roi.width = (self.roi.width as f32 * (1.0 - shrink_factor)) as i32;
    smaller_roi.height = (self.roi.height as f32 * (1.0 - shrink_factor)) as i32;

    smaller_roi.contains(pt)
  }

  pub fn roi_points(&self) -> Vector<Point2i> {
    let mut points = Vector::<Point2i>::new();
    let roi = self.roi;

    points.push(Point2i::new(roi.x,roi.y));
    points.push(Point2i::new(roi.x+roi.width, roi.y));
    points.push(Point2i::new(roi.x+roi.width, roi.y+roi.height));
    points.push(Point2i::new(roi.x,roi.y+roi.height));
    points
  }

  pub fn append_lidar_point(&mut self, lidar_point_ref: PointRef<T,U>) {
    match self.lidar_points.as_mut() {
      Some(lidar_points) => {
        lidar_points.push(lidar_point_ref);
      },
      None => {
        let mut new_lidar_points = PointIndices::<Point<T,U>>::new();
        new_lidar_points.push(lidar_point_ref);
        self.lidar_points = Some(new_lidar_points);
      }
    }
  }

}