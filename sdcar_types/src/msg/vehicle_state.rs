use crate::msg::{Msg, MsgKind};
use chrono::{DateTime, TimeZone, Utc};

use std::sync::Arc;
pub type VehicleStateRef = Arc<VehicleState>;

#[derive(Debug, Clone)]
pub struct VehicleState {
    stamp_sec: i32,
    stamp_nanosec: i64,
    timestamp: DateTime<Utc>,
    frame_id: String,
    timeref_sec: i32,
    timeref_nanosec: i64,
    timeref_stamp: DateTime<Utc>,
    vs_eng: f32,
    wheel_fl: f32,
    wheel_fr: f32,
    wheel_rl: f32,
    wheel_rr: f32,
    long_accel: f32,
    lat_accel: f32,
    yaw_rate: f32,
    sas_angle: f32,
    sas_speed: f32,
    dt_sec: f32,
    sas_radians: f32,
    vs: f32,
    v_mps: f32,
}

impl VehicleState {
    pub fn new(
        stamp_sec: i32,
        stamp_nanosec: i64,
        frame_id: String,
        timeref_sec: i32,
        timeref_nanosec: i64,
        vs_eng: f32,
        wheel_fl: f32,
        wheel_fr: f32,
        wheel_rl: f32,
        wheel_rr: f32,
        long_accel: f32,
        lat_accel: f32,
        yaw_rate: f32,
        sas_angle: f32,
        sas_speed: f32,
        dt_sec: f32,
        sas_radians: f32,
        vs: f32,
        v_mps: f32, // velocity in metres per second
    ) -> VehicleState {
        VehicleState {
            stamp_sec: stamp_sec,
            stamp_nanosec: stamp_nanosec,
            timestamp: Utc.timestamp(stamp_sec.into(), stamp_nanosec as u32),
            frame_id: frame_id,
            timeref_sec: timeref_sec,
            timeref_nanosec: timeref_nanosec,
            timeref_stamp: Utc.timestamp(timeref_sec.into(), timeref_nanosec as u32),
            vs_eng: vs_eng,
            wheel_fl: wheel_fl,
            wheel_fr: wheel_fr,
            wheel_rl: wheel_rl,
            wheel_rr: wheel_rr,
            long_accel: long_accel,
            lat_accel: lat_accel,
            yaw_rate: yaw_rate,
            sas_angle: sas_angle,
            sas_speed: sas_speed,
            dt_sec: dt_sec,
            sas_radians: sas_radians,
            vs: vs,
            v_mps: v_mps,
        }
    }
}

impl Msg for VehicleState {
    fn timestamp(&self) -> DateTime<Utc> {
        self.timestamp
    }
    fn msg_kind(&self) -> MsgKind {
        MsgKind::VehicleState(Arc::new(self.clone()))
    }
}
