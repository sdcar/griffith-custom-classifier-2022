use sdcar_types::msg::point_cloud2::PointCloud2;
use chrono::{DateTime, Utc};
use std::vec::Vec;
use std::sync::Arc;
use std::{convert::TryInto, sync::Mutex};
use crate::{point::*, HasCreationTimestamp};
use rayon::prelude::*;

pub type PointCloud2Ref = Arc::<PointCloud2>;


pub type PointIndices<T> =Vec<Arc<T>>;
pub type PointIndicesRef<T> = Arc<PointIndices<T>>;
pub type PointIndicesMutexRef<T> = Arc<Mutex<PointIndices<T>>>;

pub type PointXYZIndices<T> = PointIndices<PointXYZ<T>>;
pub type PointXYZIndicesRef<T> = Arc<PointXYZIndices<T>>;

pub type PointXYZIntensityIndices = PointXYZIndices<Intensity>;
pub type PointXYZIntensityIndicesRef = Arc<PointXYZIntensityIndices>;

pub type PointCloudRef<T> = Arc<PointCloud<T>>;
pub type PointCloudXYZIntensity = PointCloud<PointXYZIntensity>;
pub type PointCloudXYZIntensityRef = Arc<PointCloudXYZIntensity>;

// pub trait PointCloud<T> {
//   type PointCloudType;

//   fn new (pc2_msg: PointCloud2Ref) -> Self;
//   // fn point_indices_ref(&self) -> Arc<Vec<Arc<dyn Point<PointType=T>>>>;
//   fn point_indices_ref(&self) -> PointIndicesRef<T>;
//   fn num_points(&self) -> usize;
//   fn point(&self, i: usize) -> Arc<dyn Point<PointType=T>>;
//   fn timestamp(&self) -> DateTime<Utc>;
// }

// Used to access Point coordinates such as XYZ

#[derive(Debug,Clone)]
pub struct PointCloud<T>{
  pc2_msg: Option<PointCloud2Ref>,
  point_indices_ref: PointIndicesRef<T>,
}

pub trait NewPointCloud<T> {
  fn new(pc2_msg: PointCloud2Ref) -> Self;
}

impl NewPointCloud<PointXYZIntensity> for PointCloud<PointXYZIntensity> {

  fn new(pc2_msg: PointCloud2Ref) -> Self {

    let pc_rows = pc2_msg.height; // the number of rows
    let pc_row_points = pc2_msg.width; // points per rows
    let point_step = pc2_msg.point_step;

    let num_points = (pc_rows * pc_row_points) as usize;

    // let mut point_indices=PointIndices::<PointXYZIntensity>::with_capacity((pc_rows*pc_row_points) as usize);
    // TODO revisit this later - am hard coding based on what I capture
    //  [PointField { name: "x", offset: 0, datatype: 7, count: 1 }, 
    //   PointField { name: "y", offset: 4, datatype: 7, count: 1 }, 
    //   PointField { name: "z", offset: 8, datatype: 7, count: 1 }, 
    //   PointField { name: "intensity", offset: 16, datatype: 7, count: 1 }] 
    let x_offset = 0;
    let y_offset = 4;
    let z_offset = 8;
    let i_offset = 16;

    let point_build= |point_num| -> Arc<PointXYZIntensity> {
        let pos = point_num * point_step as usize;

        let x_pos = (pos+x_offset) as usize;
        let y_pos = (pos+y_offset) as usize;
        let z_pos = (pos+z_offset) as usize;
        let i_pos = (pos+i_offset) as usize;
        let x = f32::from_le_bytes(pc2_msg.data[x_pos..x_pos+4].try_into().unwrap());
        let y = f32::from_le_bytes(pc2_msg.data[y_pos..y_pos+4].try_into().unwrap());
        let z = f32::from_le_bytes(pc2_msg.data[z_pos..z_pos+4].try_into().unwrap());
        let i = f32::from_le_bytes(pc2_msg.data[i_pos..i_pos+4].try_into().unwrap());

        Arc::new(PointXYZIntensity::new(x,y,z,i))
    };

    let par_iter = (0..num_points).into_par_iter().map(point_build);
    let point_indices= par_iter.collect();

    // let point_indices=PointIndices::<PointXYZIntensity>::with_capacity((pc_rows*pc_row_points) as usize);
    // let mut append_row_points = |row_num| -> PointCloudXYZIntensity {
    //   for point_num in 0 .. pc_row_points {
    //     let pos = (row_num * pc_row_points * point_step) + (point_num * point_step);

    //     let x_pos = (pos+x_offset) as usize;
    //     let y_pos = (pos+y_offset) as usize;
    //     let z_pos = (pos+z_offset) as usize;
    //     let i_pos = (pos+i_offset) as usize;
    //     let x = f32::from_le_bytes(pc2_msg.data[x_pos..x_pos+4].try_into().unwrap());
    //     let y = f32::from_le_bytes(pc2_msg.data[y_pos..y_pos+4].try_into().unwrap());
    //     let z = f32::from_le_bytes(pc2_msg.data[z_pos..z_pos+4].try_into().unwrap());
    //     let i = f32::from_le_bytes(pc2_msg.data[i_pos..i_pos+4].try_into().unwrap());

    //     let point_ref = Box::new(PointXYZIntensity::new(x,y,z,i));
    //     // point_indices.push(point_ref);
    //     point_ref
    //   }

    // };

    // for row_num in 0 .. pc_rows {
    //     append_row_points(row_num);
    // }
    
    let point_indices_ref = PointIndicesRef::<PointXYZIntensity>::new(point_indices);
    PointCloud::<PointXYZIntensity> {pc2_msg: Some(pc2_msg), point_indices_ref}
  }
}

impl<T> HasCreationTimestamp for PointCloud<T> {
  fn timestamp(&self) -> DateTime<Utc> {
    match &self.pc2_msg {
      Some (pc2_msg) => pc2_msg.timestamp,
      None => Utc::now()
    }
  }
}


pub trait HasPointIndice<T> {
  fn num_points(&self) -> usize;
  fn point(&self, i:usize) -> Arc<T>;
  fn point_indices_ref(&self) -> PointIndicesRef<T>;
}

impl<T> HasPointIndice<T> for PointCloud<T> {

  fn num_points(&self) -> usize {
    self.point_indices_ref.len()
  }
  fn point(&self, i: usize) -> Arc<T> {
    self.point_indices_ref[i].clone()
  }
  fn point_indices_ref(&self) -> PointIndicesRef<T> {
    self.point_indices_ref.clone()
  }

}

pub trait PointCloudAccess<T>:  HasPointIndice<T> + HasCreationTimestamp + Send + Sync + std::fmt::Debug{}

impl PointCloudAccess<PointXYZIntensity> for PointCloud<PointXYZIntensity> {}


// TODO implement generic versions
// impl <T> From <PointCloud2> for PointCloud<T> {
//   fn from (pc2_msg: PointCloud2) -> Self {
//      PointCloud::<T>::new(Arc::new(pc2_msg))
//   }
// }
impl From <PointCloud2> for PointCloud<PointXYZIntensity> {
  fn from (pc2_msg: PointCloud2) -> Self {
     PointCloud::<PointXYZIntensity>::new(Arc::new(pc2_msg))
  }
}
impl From <PointCloud2Ref> for PointCloud<PointXYZIntensity> {
  fn from (pc2_msg: PointCloud2Ref) -> Self {
     PointCloud::<PointXYZIntensity>::new(pc2_msg)
  }
}

impl<T,U> From <PointIndices<Point<T,U>>> for PointCloud<Point<T,U>> 
where T: PointMeasurable,
      U: PointDataAccess
{
  fn from (point_indices: PointIndices<Point<T,U>>) -> Self {
    PointCloud::<Point::<T,U>> {pc2_msg: None, point_indices_ref: Arc::new(point_indices)}
  }
}

impl<T,U> From <PointIndicesRef<Point<T,U>>> for PointCloud<Point<T,U>> 
where T: PointMeasurable,
      U: PointDataAccess
{
  fn from (point_indices_ref: PointIndicesRef<Point<T,U>>) -> Self {
    PointCloud::<Point::<T,U>> {pc2_msg: None, point_indices_ref: point_indices_ref}
  }
}