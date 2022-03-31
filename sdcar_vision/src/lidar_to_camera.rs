extern crate nalgebra as na;
extern crate opencv;

use na::{Matrix3, Matrix3x4, Matrix4, Point2, Point3};

use crate::stereo_calibration::StereoCalibration;

#[derive(Debug, Clone)]
pub struct LiDAR2Camera {
    stereo_calibration: StereoCalibration,
    p: Matrix3x4<f64>,   // camera matrix
    l2c: Matrix3x4<f64>, // rigid transform from lidar coord to reference camera coord
    r0: Matrix3<f64>,    // rotation from reference camera coord to rect camera coord

    p_r0_rt: Matrix3x4<f64>, // pre computed calculations used for lidar (3d) to camera (2d)
}

impl LiDAR2Camera {
    pub fn new(
        stereo_calibration: StereoCalibration,
        p: Matrix3x4<f64>,
        l2c: Matrix3x4<f64>,
        r0: Matrix3<f64>,
    ) -> Self {
        let p_r0_rt = Self::calc_p_r0_rt(&p, &l2c, &r0);
        Self {
            stereo_calibration,
            p,
            l2c,
            r0,
            p_r0_rt
        }
    }

    pub fn from_left_camera(stereo_calibration: StereoCalibration) -> Self {
        let p = stereo_calibration.p1();

        // left camera is -25cm
        let l2c = Matrix3x4::<f64>::new(
            0.0, -1.0, 0.0, 0.25,
            0.0, 0.0, -1.0, -0.125,
            1.0, 0.0, 0.0, -0.115
        );
        let r0 = stereo_calibration.r2();
        let p_r0_rt = Self::calc_p_r0_rt(&p, &l2c, &r0);
        Self {
            stereo_calibration,
            p,
            l2c,
            r0,
            p_r0_rt
        }
    }
    pub fn from_right_camera(stereo_calibration: StereoCalibration) -> Self {
        let p = stereo_calibration.p2();
        // right camera is +25cm
        let l2c = Matrix3x4::<f64>::new(
            0.0, -1.0, 0.0, -0.25,
            0.0, 0.0, -1.0, -0.125,
            1.0, 0.0, 0.0, -0.115
        );
        let r0 = stereo_calibration.r1();
        let p_r0_rt = Self::calc_p_r0_rt(&p, &l2c, &r0);
        Self {
            stereo_calibration,
            p,
            l2c,
            r0,
            p_r0_rt
        }
    }


    fn calc_p_r0_rt(p: &Matrix3x4<f64>, l2c: &Matrix3x4<f64>, r0: &Matrix3<f64>) -> Matrix3x4<f64> {

      println!("P {}", &p);
      println!("R0 {}", &r0);
      println!("R|t {}", &l2c);

      let p_r0 = p * r0.to_homogeneous();

      let l2c_h = Matrix4::<f64>::new(
          l2c[(0, 0)],
          l2c[(0, 1)],
          l2c[(0, 2)],
          l2c[(0, 3)],
          l2c[(1, 0)],
          l2c[(1, 1)],
          l2c[(1, 2)],
          l2c[(1, 3)],
          l2c[(2, 0)],
          l2c[(2, 1)],
          l2c[(2, 2)],
          l2c[(2, 3)],
          0.0,
          0.0,
          0.0,
          1.0,
      );
      let p_r0_rt: Matrix3x4<f64> = p_r0 * l2c_h;
    //   println!("p_r0_rt {}", p_r0_rt);
      p_r0_rt
    }

    pub fn project_lidar_to_image(&self, x_3d: Point3<f64>) -> Option<Point2<f64>> {
       let p_r0_rt_x = self.p_r0_rt * x_3d.to_homogeneous();
       Point2::from_homogeneous(p_r0_rt_x)
    }
}
