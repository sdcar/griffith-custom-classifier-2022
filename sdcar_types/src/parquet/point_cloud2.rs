extern crate parquet;

use parquet::record::{ListAccessor, RowAccessor};

use crate::msg::point_cloud2::{PointCloud2, PointField};
use std::vec::Vec;

impl From<parquet::record::Row> for PointCloud2 {
    fn from(row: parquet::record::Row) -> Self {
        let header = row.get_group(1).unwrap();
        let stamp = header.get_group(0).unwrap();
        let sec = stamp.get_int(0).unwrap();
        let nanosec = stamp.get_uint(1).unwrap();
        let frame_id = header.get_string(1).unwrap().to_string();
        let height = row.get_uint(2).unwrap();
        let width = row.get_uint(3).unwrap();
        let fields_row = row.get_list(4).unwrap();
        let mut fields: Vec<PointField> = Vec::with_capacity(fields_row.len());
        for i in 0..fields_row.len() {
            let field_group = fields_row.get_group(i).unwrap();

            let field = PointField::new(
                field_group.get_string(0).unwrap().to_string(),
                field_group.get_uint(1).unwrap(),
                field_group.get_ubyte(2).unwrap(),
                field_group.get_uint(3).unwrap(),
            );
            fields.push(field);
        }
        let is_bigendian = row.get_bool(5).unwrap();
        let point_step = row.get_uint(6).unwrap();
        let row_step = row.get_uint(7).unwrap();
        let data = row.get_bytes(8).unwrap().data();

        let is_dense = row.get_bool(9).unwrap();

        PointCloud2::new(
            sec,
            nanosec,
            frame_id,
            height,
            width,
            fields,
            is_bigendian,
            point_step,
            row_step,
            data.to_vec(),
            is_dense,
        )
    }
}
