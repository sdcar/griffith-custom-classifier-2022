use crate::msg::{Msg, MsgKind};
use chrono::{DateTime, TimeZone, Utc};

use std::sync::Arc;

pub type ImageRef = Arc<Image>;

#[derive(Debug, Clone)]
pub struct Image {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub timestamp: DateTime<Utc>,
    pub frame_id: String,
    pub height: u32,
    pub width: u32,
    pub encoding: String,
    pub is_bigendian: bool,
    pub step: u32,
    pub image_oid: String,
}

impl Image {
    pub fn new(
        stamp_sec: i32,
        stamp_nanosec: u32,
        frame_id: String,
        height: u32,
        width: u32,
        encoding: String,
        is_bigendian: bool,
        step: u32,
        image_oid: String,
    ) -> Self {
        Self {
            stamp_sec,
            stamp_nanosec,
            timestamp: Utc.timestamp(stamp_sec.into(), stamp_nanosec),
            frame_id,
            height,
            width,
            encoding,
            is_bigendian,
            step,
            image_oid,
        }
    }
}

impl Msg for Image {
    fn timestamp(&self) -> DateTime<Utc> {
        self.timestamp
    }
    fn msg_kind(&self) -> MsgKind {
        MsgKind::Image(Arc::new(self.clone()))
    }
}
