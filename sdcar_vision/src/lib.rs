pub mod stereo_rectify;
pub mod object_detection_2d;
pub mod bounding_box;
pub mod image_frame;
pub mod camera;
pub mod stereo_calibration;
pub mod lidar_to_camera;
pub mod fusion;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
