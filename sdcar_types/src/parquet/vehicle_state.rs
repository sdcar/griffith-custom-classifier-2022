extern crate parquet;

use parquet::record::RowAccessor;

use crate::msg::vehicle_state::VehicleState;

impl From<parquet::record::Row> for VehicleState {
    fn from(row: parquet::record::Row) -> Self {
        let header = row.get_group(1).unwrap();
        let stamp = header.get_group(0).unwrap();
        let sec = stamp.get_int(0).unwrap();
        let nanosec = stamp.get_long(1).unwrap();
        let frame = header.get_string(1).unwrap().to_string();
        let timeref = row.get_group(2).unwrap();
        let timeref_sec = timeref.get_int(0).unwrap();
        let timeref_nanosec = timeref.get_long(1).unwrap();
        let vs_eng = row.get_float(3).unwrap();
        let wheel_fl = row.get_float(4).unwrap();
        let wheel_fr = row.get_float(5).unwrap();
        let wheel_rl = row.get_float(6).unwrap();
        let wheel_rr = row.get_float(7).unwrap();
        let long_accel = row.get_float(8).unwrap();
        let lat_accel = row.get_float(9).unwrap();
        let yaw_rate = row.get_float(10).unwrap();
        let sas_angle = row.get_float(11).unwrap();
        let sas_speed = row.get_float(12).unwrap();
        let dt_sec = row.get_float(13).unwrap();
        let sas_radian = row.get_float(14).unwrap();
        let vs = row.get_float(15).unwrap();
        let v_mps = row.get_float(16).unwrap();

        VehicleState::new(
            sec,
            nanosec,
            frame,
            timeref_sec,
            timeref_nanosec,
            vs_eng,
            wheel_fl,
            wheel_fr,
            wheel_rl,
            wheel_rr,
            long_accel,
            lat_accel,
            yaw_rate,
            sas_angle,
            sas_speed,
            dt_sec,
            sas_radian,
            vs,
            v_mps,
        )
    }
}
