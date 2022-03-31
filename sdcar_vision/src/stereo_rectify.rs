extern crate nalgebra as na;
extern crate opencv;

use opencv::calib3d::{get_optimal_new_camera_matrix, init_undistort_rectify_map};
use opencv::core::{Mat, Rect, Size};
use opencv::imgproc::{remap, INTER_NEAREST};
use opencv::prelude::*;

use crate::stereo_calibration::StereoCalibration;

#[derive(Debug, Clone)]
pub struct StereoRectify {
    stereo_calibration: StereoCalibration,
    size: Size,
    //TODO if using opencv Mat in struct cant parallelise
    undistortion_map_l: Mat,  // Undistortion Map Left
    undistortion_map_r: Mat,  // Undistortion Map Right
    rectification_map_l: Mat, // Rectification Map Left
    rectification_map_r: Mat, // Rectification Map Right
}

impl StereoRectify {
    pub fn new(stereo_calibration: StereoCalibration, size: Size) -> Self {
        let mut undistortion_map_l = Mat::default();
        let mut undistortion_map_r = Mat::default();
        let mut rectification_map_l = Mat::default();
        let mut rectification_map_r = Mat::default();

        let mut new_camera_matrix_l = Mat::default();
        let mut new_camera_matrix_r = Mat::default();

        let mut valid_pix_roi_l = Rect::default();
        let mut valid_pix_roi_r = Rect::default();

        let m1type = opencv::core::CV_32FC1;

        let new_camera_matrix_l_result = get_optimal_new_camera_matrix(
            &stereo_calibration.k1_mat(),
            &stereo_calibration.d1_mat(),
            size,
            -1.0,
            size,
            &mut valid_pix_roi_l,
            false,
        );
        match new_camera_matrix_l_result {
            Ok(mat) => new_camera_matrix_l = mat,
            Err(e) => println!("error left new camera matrix: {:#?}", e),
        };

        let new_camera_matrix_r_result = get_optimal_new_camera_matrix(
            &stereo_calibration.k2_mat(),
            &stereo_calibration.d2_mat(),
            size,
            -1.0,
            size,
            &mut valid_pix_roi_r,
            false,
        );
        match new_camera_matrix_r_result {
            Ok(mat) => new_camera_matrix_r = mat,
            Err(e) => println!("error right new camera matrix: {:#?}", e),
        };
        println!("new_camera_matrix_l: {:?}", &new_camera_matrix_l.clone());
        println!("new_camera_matrix_r: {:?}", &new_camera_matrix_r.clone());

        let left_init_result = init_undistort_rectify_map(
            &stereo_calibration.k1_mat(),
            &stereo_calibration.d1_mat(),
            &stereo_calibration.r1_mat(),
            &new_camera_matrix_l,
            size,
            m1type,
            &mut undistortion_map_l,
            &mut rectification_map_l,
        );
        match left_init_result {
            Ok(_) => {}
            Err(e) => println!("error left stereo rectify init: {:?}", e),
        };

        let right_init_result = init_undistort_rectify_map(
            &stereo_calibration.k2_mat(),
            &stereo_calibration.d2_mat(),
            &stereo_calibration.r2_mat(),
            &new_camera_matrix_r,
            size,
            m1type,
            &mut undistortion_map_r,
            &mut rectification_map_r,
        );
        match right_init_result {
            Ok(_) => {}
            Err(e) => println!("error right stereo rectify init: {:?}", e),
        };

        Self {
            stereo_calibration,
            size,
            undistortion_map_l,
            undistortion_map_r,
            rectification_map_l,
            rectification_map_r,
        }
    }

    pub fn rectify_left(&self, frame: Mat) -> Mat {
        let mut rectified = Mat::default();
        let result = remap(
            &frame,
            &mut rectified,
            &self.undistortion_map_l,
            &self.rectification_map_l,
            INTER_NEAREST,
            opencv::core::BORDER_TRANSPARENT,
            opencv::core::Scalar::new(0., 0., 0., 0.),
        );
        match result {
            Ok(_) => {}
            Err(e) => println!("error rectify left failed: {:?}", e),
        }
        rectified
    }

    pub fn rectify_right(&self, frame: Mat) -> Mat {
        let mut rectified = Mat::default();
        let result = remap(
            &frame,
            &mut rectified,
            &self.undistortion_map_r,
            &self.rectification_map_r,
            INTER_NEAREST,
            opencv::core::BORDER_TRANSPARENT,
            opencv::core::Scalar::new(0., 0., 0., 0.),
        );
        match result {
            Ok(_) => {}
            Err(e) => println!("error rectify right failed: {:?}", e),
        }
        rectified
    }
}
