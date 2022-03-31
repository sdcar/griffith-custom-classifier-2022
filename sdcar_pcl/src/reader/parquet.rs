extern crate sdcar_types;
use sdcar_types::reader::parquet::*;
use sdcar_types::reader::combined::*;
use sdcar_types::msg::MsgKind;

use std::sync::Arc;

use crate::prelude::*;

pub struct PointCloudXYZIntensityRowIter<'a> {
  point_cloud2_row_iter: PointCloud2ParquetRowIter<'a>,
}

impl <'a> PointCloudXYZIntensityRowIter<'a>{
  pub fn new(path:&'a str) -> Self{
    let point_cloud2_row_iter = PointCloud2ParquetRowIter::new(path);
    Self {point_cloud2_row_iter}
  }
}

impl <'a> Iterator for PointCloudXYZIntensityRowIter <'a> {
  type Item = PointCloud<PointXYZIntensity>;

  fn next(&mut self) -> Option<Self::Item> {
    match self.point_cloud2_row_iter.next() {
      Some(pc2_msg) => {
        Some(PointCloud::<PointXYZIntensity>::from(pc2_msg))
      },
      None => None,
    }
  }
}

pub struct ImageRowIter<'a> {
  image_path: &'a str,
  image_row_iter: ImageParquetRowIter<'a>,
}

impl <'a> ImageRowIter<'a>{
  pub fn new(path:&'a str, image_path:&'a str) -> Self{
    let image_row_iter = ImageParquetRowIter::new(path);
    Self {image_row_iter, image_path}
  }
}

impl <'a> Iterator for ImageRowIter<'a>{
  type Item = ImageFile;

  fn next(&mut self) -> Option<Self::Item> {
    match self.image_row_iter.next() {
      Some(image ) => {
        Some(ImageFile::from(image))
      },
      None => None,
    }
  }
}


pub struct PointCloudXYZIntensityImageRowIter<'a>
{
  pc2_image_row_iter: Reader2<PointCloud2ParquetRowIter<'static>, ImageParquetRowIter<'static>>,
  image_data_path: &'a str,
}
impl <'a> PointCloudXYZIntensityImageRowIter<'a>{
  pub fn new(point_cloud2_row_iter:PointCloud2ParquetRowIter<'static>, image_row_iter:ImageParquetRowIter<'static>, image_data_path:&'a str) -> Self{
    // let mut point_cloud2_row_iter = PointCloud2ParquetRowIter::new(pc2_file);
    // let mut image_row_iter = ImageParquetRowIter::new(image_file);
    let pc2_image_row_iter = Reader2::new(point_cloud2_row_iter, image_row_iter);
    Self {pc2_image_row_iter,image_data_path}
  }
  pub fn image_data_path(&self) -> &str {
    self.image_data_path.clone()
  }
}

impl <'a> Iterator for PointCloudXYZIntensityImageRowIter<'a>{

  type Item = DataKind<XYZ,Intensity>;

  fn next(&mut self) -> Option<Self::Item> {
    if let Some(next_row) = self.pc2_image_row_iter.next() {
      match next_row.msg_kind() {
        MsgKind::PointCloud2(pc2_ref)=> {
          let pc = PointCloudXYZIntensity::from(pc2_ref);
          Some(DataKind::<XYZ,Intensity>::PointCloud(Arc::new(pc)))
        },
        MsgKind::Image(image_ref) => {
          let mut image_file = ImageFile::from(image_ref);
          image_file.path(self.image_data_path.to_string());
          Some(DataKind::<XYZ,Intensity>::ImageFile(Arc::new(image_file)))
        },
        _ => None
      }
    } else {
      None
    }
  }
}