use chrono::{DateTime, TimeZone, Utc};

use crate::msg::{Msg, MsgKind};
use std::vec::Vec;

use std::sync::Arc;

pub type PointCloud2Ref = Arc<PointCloud2>;

#[derive(Debug, Clone)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

impl PointField {
    pub fn new(name: String, offset: u32, datatype: u8, count: u32) -> PointField {
        PointField {
            name,
            offset,
            datatype,
            count,
        }
    }
}

#[derive(Debug, Clone)]
pub struct PointCloud2 {
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub timestamp: DateTime<Utc>,
    pub frame_id: String,
    pub height: u32,
    pub width: u32,
    pub fields: Vec<PointField>,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: Vec<u8>,
    pub is_dense: bool,
}

impl PointCloud2 {
    pub fn new(
        stamp_sec: i32,
        stamp_nanosec: u32,
        frame_id: String,
        height: u32,
        width: u32,
        fields: Vec<PointField>,
        is_bigendian: bool,
        point_step: u32,
        row_step: u32,
        data: Vec<u8>,
        is_dense: bool,
    ) -> PointCloud2 {
        PointCloud2 {
            stamp_sec,
            stamp_nanosec,
            timestamp: Utc.timestamp(stamp_sec.into(), stamp_nanosec),
            frame_id,
            height,
            width,
            fields,
            is_bigendian,
            point_step,
            row_step,
            data,
            is_dense,
        }
    }
}

impl Msg for PointCloud2 {
    fn timestamp(&self) -> DateTime<Utc> {
        self.timestamp
    }
    fn msg_kind(&self) -> MsgKind {
        MsgKind::PointCloud2(Arc::new(self.clone()))
    }
}
