pub mod point;
pub mod point_cloud;
pub mod bounding_box;
pub mod filters;
pub mod segmentation;
pub mod search;
pub mod reader;
pub mod prelude;
pub mod image;

use chrono::{DateTime, Utc};

use point::{Point,PointRef,PointMeasurable,PointDataAccess};
use point_cloud::PointCloudRef;
use bounding_box::BoundingBoxRef;
use image::ImageFileRef;


pub trait HasCreationTimestamp {
  fn timestamp(&self) -> DateTime<Utc>; 
}

#[derive(Debug, Clone)]
pub enum DataKind <T,U>
where T: PointMeasurable,
      U: PointDataAccess
{
    Point(PointRef<T,U>),
    PointCloud(PointCloudRef<Point<T,U>>),
    BoundingBox(BoundingBoxRef<T,U>),
    ImageFile(ImageFileRef)
}

#[cfg(test)]
mod tests {

    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
