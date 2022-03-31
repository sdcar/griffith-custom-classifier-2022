extern crate nalgebra as na;
extern crate opencv;

use std::path::Path;
use opencv::core::{FileStorage, FileStorage_Mode};
use opencv::core::{Mat, Vector};
use opencv::prelude::*;

use na::{Matrix3, Matrix3x4, Matrix4, Vector3, Vector5};

#[derive(Debug, Clone)]
pub struct StereoCalibration {
    pub filename: String,
    k1: Matrix3<f64>, // Camera Matrix - Left
    k2: Matrix3<f64>, // Camera Matrix - right
    d1: Vector5<f64>, // distortion coefficients - left
    d2: Vector5<f64>, // distortion coefficients - right
    r: Matrix3<f64>,  // rotation matrix
    t: Vector3<f64>,  // translation vector
    e: Matrix3<f64>,  // essential matrix
    f: Matrix3<f64>,  // fundamental matrix
    r1: Matrix3<f64>, // rectification translation matrix - left
    r2: Matrix3<f64>, // rectification translation matrix - right
    p1: Matrix3x4<f64>, // projection matrix - left
    p2: Matrix3x4<f64>, // projection matrix - right
    q: Matrix4<f64>,  // disparty to depth matrix
}

impl StereoCalibration {
    pub fn new(filename: &str) -> Self {
        let flags = FileStorage_Mode::READ as i32 + FileStorage_Mode::FORMAT_YAML as i32;
        let fs = FileStorage::new(filename, flags, "utf-8").unwrap();

        let k1 = Self::as_matrix3(&fs.get("K1").unwrap().mat().unwrap());
        let k2 = Self::as_matrix3(&fs.get("K2").unwrap().mat().unwrap());
        let d1 = Self::as_vector5(&fs.get("D1").unwrap().mat().unwrap());
        let d2 = Self::as_vector5(&fs.get("D2").unwrap().mat().unwrap());
        let r = Self::as_matrix3(&fs.get("R").unwrap().mat().unwrap());
        let t = Self::as_vector3(&fs.get("T").unwrap().mat().unwrap());
        let e = Self::as_matrix3(&fs.get("E").unwrap().mat().unwrap());
        let f = Self::as_matrix3(&fs.get("F").unwrap().mat().unwrap());
        let r1 = Self::as_matrix3(&fs.get("R1").unwrap().mat().unwrap());
        let r2 = Self::as_matrix3(&fs.get("R2").unwrap().mat().unwrap());
        let p1 = Self::as_matrix3x4(&fs.get("P1").unwrap().mat().unwrap());
        let p2 = Self::as_matrix3x4(&fs.get("P2").unwrap().mat().unwrap());
        let q = Self::as_matrix4(&fs.get("Q").unwrap().mat().unwrap());
        Self {
            filename: filename.to_string(),
            k1,
            k2,
            d1,
            d2,
            r,
            t,
            e,
            f,
            r1,
            r2,
            p1,
            p2,
            q,
        }
    }

    pub fn k1(&self) -> Matrix3<f64> {
        self.k1
    }
    pub fn k1_mat(&self) -> Mat {
        Mat::from_slice(self.k1.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn d1(&self) -> Vector5<f64> {
        self.d1
    }
    pub fn d1_mat(&self) -> Mat {
        Mat::from_slice(self.d1.as_slice()).unwrap()
    }
    pub fn k2(&self) -> Matrix3<f64> {
        self.k2
    }
    pub fn k2_mat(&self) -> Mat {
        Mat::from_slice(self.k2.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn d2(&self) -> Vector5<f64> {
        self.d2
    }
    pub fn d2_mat(&self) -> Mat {
        Mat::from_slice(self.d2.as_slice()).unwrap()
    }
    pub fn r(&self) -> Matrix3<f64> {
        self.r
    }
    pub fn r_mat(&self) -> Mat {
        Mat::from_slice(self.r.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn t(&self) -> Vector3<f64> {
        self.t
    }
    pub fn t_mat(&self) -> Mat {
        Mat::from_slice(self.t.as_slice()).unwrap()
    }
    pub fn e(&self) -> Matrix3<f64> {
        self.e
    }
    pub fn e_mat(&self) -> Mat {
        Mat::from_slice(self.e.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn f(&self) -> Matrix3<f64> {
        self.f
    }
    pub fn f_mat(&self) -> Mat {
        Mat::from_slice(self.f.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn r1(&self) -> Matrix3<f64> {
        self.r1
    }
    pub fn r1_mat(&self) -> Mat {
        Mat::from_slice(self.r1.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn r2(&self) -> Matrix3<f64> {
        self.r2
    }
    pub fn r2_mat(&self) -> Mat {
        Mat::from_slice(self.r2.as_slice()).unwrap().reshape(1,3).unwrap()
    }
    pub fn p1(&self) -> Matrix3x4<f64> {
        self.p1
    }
    pub fn p1_mat(&self) -> Mat {
        Mat::from_slice(self.p1.as_slice()).unwrap().reshape(1,4).unwrap()
    }
    pub fn p2(&self) -> Matrix3x4<f64> {
        self.p2
    }
    pub fn p2_mat(&self) -> Mat {
        Mat::from_slice(self.p2.as_slice()).unwrap().reshape(1,4).unwrap()
    }
    pub fn q(&self) -> Matrix4<f64> {
        self.q
    }
    pub fn q_mat(&self) -> Mat {
        Mat::from_slice(self.q.as_slice()).unwrap().reshape(1,4).unwrap()
    }

    fn as_matrix3(m: &Mat) -> Matrix3<f64> {
        let m_data = m.data() as *const f64;
        let mut m_na = Matrix3::<f64>::zeros();
        unsafe {
            let m_slice = std::slice::from_raw_parts(m_data, m_na.len());
            m_na.copy_from(&Matrix3::<f64>::from_row_slice(&m_slice));
        }
        m_na.clone_owned()
    }
    fn as_matrix3x4(m: &Mat) -> Matrix3x4<f64> {
        let m_data = m.data() as *const f64;
        let mut m_na = Matrix3x4::<f64>::zeros();
        unsafe {
            let m_slice = std::slice::from_raw_parts(m_data, m_na.len());

            m_na.copy_from(&Matrix3x4::<f64>::from_row_slice(&m_slice));
        }
        m_na.clone_owned()
    }
    fn as_matrix4(m: &Mat) -> Matrix4<f64> {
        let m_data = m.data() as *const f64;
        let mut m_na = Matrix4::<f64>::zeros();
        unsafe {
            let m_slice = std::slice::from_raw_parts(m_data, m_na.len());
            m_na.copy_from(&Matrix4::<f64>::from_row_slice(&m_slice));
        }
        m_na.clone_owned()
    }
    fn as_vector5(m: &Mat) -> Vector5<f64> {
        let m_data = m.data() as *const f64;
        let mut v_na = Vector5::<f64>::zeros();
        unsafe {
            let m_slice = std::slice::from_raw_parts(m_data, v_na.len());
            v_na.copy_from_slice(&m_slice);
        }
        v_na.clone_owned()
    }
    fn as_vector3(m: &Mat) -> Vector3<f64> {
        let m_data = m.data() as *const f64;
        let mut v_na = Vector3::<f64>::zeros();
        unsafe {
            let m_slice = std::slice::from_raw_parts(m_data, v_na.len());
            v_na.copy_from_slice(&m_slice);
        }
        v_na.clone_owned()
    }
}
