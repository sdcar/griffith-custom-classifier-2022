use chrono::{DateTime, TimeZone, Utc};

extern crate coord_transforms;
extern crate nalgebra as na;

use coord_transforms::prelude::*;
use std::vec::Vec;

use crate::msg::{Msg, MsgKind};

use std::sync::Arc;

pub type NavSatFixRef = Arc<NavSatFix>;

#[derive(Debug, Clone)]
pub struct NavSatFix {
    pub stamp_sec: i32,
    pub stamp_nanosec: i64,
    pub timestamp: DateTime<Utc>,
    pub frame_id: String,
    pub status: i8,
    pub service: u16,
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub position_covariance: [f64; 9],
    pub position_covariance_type: u8,
}

impl NavSatFix {
    pub fn new(
        stamp_sec: i32,
        stamp_nanosec: i64,
        frame_id: String,
        status: i8,
        service: u16,
        latitude: f64,
        longitude: f64,
        altitude: f64,
        position_covariance: [f64; 9],
        position_covariance_type: u8,
    ) -> NavSatFix {
        NavSatFix {
            stamp_sec,
            stamp_nanosec,
            timestamp: Utc.timestamp(stamp_sec.into(), stamp_nanosec as u32),
            frame_id,
            status,
            service,
            latitude,
            longitude,
            altitude,
            position_covariance,
            position_covariance_type,
        }
    }
    pub fn lla_vec(&self) -> Vec<f64> {
        vec![
            self.latitude.to_radians(),
            self.longitude.to_radians(),
            self.altitude,
        ]
    }
    pub fn ecef(&self, geo_ellipsoid: &geo_ellipsoid::geo_ellipsoid) -> na::Vector3<f64> {
        //Convert to Earth-Centered Earth-Fixed (ECEF)

        // need to do this as different versions of nalgebra being used
        let lla = Vector3::from_vec(self.lla_vec());
        let ecef = geo::lla2ecef(&lla, &geo_ellipsoid);
        na::Vector3::new(ecef[0], ecef[1], ecef[2])
    }

    // becuase different version of nalgebra
    fn na_convert(&self, o: &na::Vector3<f64>) -> Vector3<f64> {
        Vector3::new(o[0], o[1], o[2])
    }

    pub fn ned(
        &self,
        geo_ellipsoid: &geo_ellipsoid::geo_ellipsoid,
        lla_origin: &na::Vector3<f64>,
    ) -> na::Vector3<f64> {
        // convert to ned
        let lla = Vector3::from_vec(self.lla_vec());
        let ned = geo::lla2ned(&self.na_convert(&lla_origin), &lla, &geo_ellipsoid);
        na::Vector3::new(ned[0], ned[1], ned[2])
    }

    pub fn enu(
        &self,
        geo_ellipsoid: &geo_ellipsoid::geo_ellipsoid,
        lla_origin: &na::Vector3<f64>,
    ) -> na::Vector3<f64> {
        // convert to enu
        let lla = Vector3::from_vec(self.lla_vec());
        let enu = geo::lla2enu(&self.na_convert(&lla_origin), &lla, &geo_ellipsoid);
        na::Vector3::new(enu[0], enu[1], enu[2])
    }
}

impl Msg for NavSatFix {
    fn timestamp(&self) -> DateTime<Utc> {
        self.timestamp
    }
    fn msg_kind(&self) -> MsgKind {
        MsgKind::NavSatFix(Arc::new(self.clone()))
    }
}
