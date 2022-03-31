extern crate parquet;

use parquet::record::{ListAccessor, RowAccessor};

use crate::msg::nav_sat_fix::NavSatFix;

impl From<parquet::record::Row> for NavSatFix {
    fn from(row: parquet::record::Row) -> Self {
        let header = row.get_group(1).unwrap();
        let stamp = header.get_group(0).unwrap();
        let sec = stamp.get_int(0).unwrap();
        let nanosec = stamp.get_long(1).unwrap();
        let frame = header.get_string(1).unwrap().to_string();
        let status_group = row.get_group(2).unwrap();
        let status = status_group.get_byte(0).unwrap();
        let service = status_group.get_ushort(1).unwrap();
        let latitude = row.get_double(3).unwrap();
        let longitude = row.get_double(4).unwrap();
        let altitude = row.get_double(5).unwrap();
        let mut covar: [f64; 9] = [0.0; 9];
        let covar_row = row.get_list(6).unwrap();
        for i in 0..covar_row.len() {
            covar[i] = covar_row.get_double(i).unwrap();
        }
        let covar_type = row.get_ubyte(7).unwrap();

        NavSatFix::new(
            sec, nanosec, frame, status, service, latitude, longitude, altitude, covar, covar_type,
        )
    }
}
