extern crate sdcar_pcl;
extern crate sdcar_vision;
extern crate opencv;

use sdcar_pcl::prelude::*;

use sdcar_vision::bounding_box::{ImageBoundingBox,ImageBoundingBoxes};
use opencv::core::Point2i;

pub fn find_lidar_cluster_ego_front<T, U>(
    lidar_boundinging_boxes: Option<BoundingBoxesRef<T, U>>,
) -> Option<BoundingBoxRef<T, U>>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    let mut bb_found: Option<BoundingBoxRef<T, U>> = None;

    if let Some(bbes) = lidar_boundinging_boxes {
        for bb in bbes.iter().cloned() {
            let point_min = bb.point_min();
            let point_max = bb.point_max();

            // if cluster is directly in front of ego return it
            let x = point_min.point().array()[0];
            let y = if point_min.point().array()[1] < 0.0 {
                point_max.point().array()[1]
            } else {
                point_min.point().array()[1]
            };
            if x > 0.0 && x < 25.0 && y > -1.5 && y < 1.5 {
                bb_found = Some(bb);
                break;
            }
        }
    }

    bb_found
}

pub fn find_image_object_ego_front<T,U>(
    image_bounding_boxes: Option<ImageBoundingBoxes<T, U>>
) -> Option<ImageBoundingBox<T,U>>
where
    T: PointMeasurable,
    U: PointDataAccess,

{
  let mut bb_found: Option<ImageBoundingBox<T, U>> = None;
//   let pt = Point2i::new(647,316);
  let pt = Point2i::new(1150,800);

  if let Some(bbes) = image_bounding_boxes {
      for bb in bbes.iter().cloned() {
          if bb.contains(pt) {
              bb_found = Some(bb);
              break;
          }
          println!("{:?}", bb);
      }
  }

  bb_found

}
