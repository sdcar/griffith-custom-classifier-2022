extern crate sdcar_pcl;
extern crate sdcar_vision;

extern crate nalgebra as na;
extern crate opencv;

use opencv::core::*;
use opencv::highgui;
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::imgproc::{get_text_size, FILLED, FONT_ITALIC, LINE_4};
use opencv::prelude::*;
use opencv::Result;

use sdcar_pcl::prelude::*;
use std::sync::Arc;

use sdcar_vision::bounding_box::ImageBoundingBoxesRef;
use sdcar_vision::camera::*;
use sdcar_calib::CalibFrame;

use sdcar_calib::ego::{find_lidar_cluster_ego_front,find_image_object_ego_front};

pub fn fuse_image_lidar<T, U>(
    camera_img: Mat,
    lidar_points: Option<PointCloudRef<sdcar_pcl::point::Point<T,U>>>,
    image_bounding_boxes: Option<ImageBoundingBoxesRef<T, U>>,
    lidar_bounding_boxes: Option<BoundingBoxesRef<T, U>>,
    clusters: ClustersRef<T, U>,
) -> CalibFrame<T, U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    // this version looks for a vehicle in front of ego and then projects clustered lidar points onto the camera plane image

    let target_lidar_bb = find_lidar_cluster_ego_front(lidar_bounding_boxes.clone());

    let target_lidar_points = if let Some(bb) = target_lidar_bb.clone() {
        Some(clusters[bb.cluster_id.unwrap_or_default() as usize].clone())
    } else {
        None
    };

    let target_image_bb = find_image_object_ego_front(Some(image_bounding_boxes.clone().unwrap().read().unwrap().clone()));

    let image_frame = CalibFrame::<T, U>::new(
        camera_img,
        lidar_points,
        lidar_bounding_boxes,
        target_lidar_points,
        target_lidar_bb,
        image_bounding_boxes,
        target_image_bb,
    );

    image_frame
}
