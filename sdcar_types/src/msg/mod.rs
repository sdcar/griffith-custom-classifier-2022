pub mod image;
pub mod nav_sat_fix;
pub mod point_cloud2;
pub mod vehicle_state;

use image::ImageRef;
use nav_sat_fix::NavSatFixRef;
use point_cloud2::PointCloud2Ref;
use vehicle_state::VehicleStateRef;

use chrono::{DateTime, Utc};

pub trait Msg {
    fn timestamp(&self) -> DateTime<Utc>;
    fn msg_kind(&self) -> MsgKind;
}

#[derive(Debug, Clone)]
pub enum MsgKind {
    NavSatFix(NavSatFixRef),
    VehicleState(VehicleStateRef),
    PointCloud2(PointCloud2Ref),
    Image(ImageRef),
}
