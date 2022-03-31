extern crate sdcar_types;
use sdcar_types::msg::image::{Image,ImageRef};
use std::sync::Arc;
use chrono::{DateTime, Utc};

use crate::HasCreationTimestamp;

pub type ImageFileRef = Arc<ImageFile>;

#[derive(Debug, Clone)]
pub enum ImageKind {
  JPG,
  WEBP,
}

#[derive(Debug, Clone)]
pub enum EncodingKind{
  BGR8,
}

#[derive(Debug, Clone)]
pub struct ImageFile{
  kind: ImageKind,
  path: Option<String>,
  oid: String,
  file_name: Option<String>,
  height: usize,
  width: usize,
  encoding: EncodingKind,
  timestamp: DateTime<Utc>
}

impl ImageFile {
  pub fn new(kind:ImageKind, oid:String, height: usize, width: usize, encoding:EncodingKind, timestamp: DateTime<Utc>) -> Self {
    Self {kind,path:None,oid,file_name:None,height,width,encoding, timestamp}
  }
  pub fn path(&mut self, path:String) {
    self.path = Some(path.clone());
    // TODO change the extenstion based on the image kind
    // let file_name= format!("{}/{}.jpg",path,self.oid);
    let file_name= format!("{}/{}.webp",path,self.oid);

    self.file_name = Some(file_name);
  }
  pub fn file_name(&self) -> String {
    self.file_name.clone().unwrap_or_default()
  }
  pub fn oid(&self) -> String {
    self.oid.clone()
  }
}

impl HasCreationTimestamp for ImageFile {
  fn timestamp(&self) -> DateTime<Utc> {
    self.timestamp
  }
}

impl From<Image> for ImageFile{
  fn from(image: Image) -> Self{

    let image_oid = image.image_oid;
    let timestamp = image.timestamp;

    ImageFile::new(ImageKind::WEBP, image_oid, image.height as usize, image.width as usize,EncodingKind::BGR8, timestamp)
  }
}

impl From<ImageRef> for ImageFile{
  fn from(image: ImageRef) -> Self{
    let image_oid = image.image_oid.to_owned();
    let timestamp = image.timestamp.to_owned();
    ImageFile::new(ImageKind::WEBP, image_oid, image.height as usize, image.width as usize,EncodingKind::BGR8, timestamp)
  }
}