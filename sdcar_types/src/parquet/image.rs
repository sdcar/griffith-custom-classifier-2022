extern crate parquet;

use parquet::record::RowAccessor;

use crate::msg::image::Image;

impl From<parquet::record::Row> for Image {
    fn from(row: parquet::record::Row) -> Self {
        let header = row.get_group(1).unwrap();
        let stamp = header.get_group(0).unwrap();
        let sec = stamp.get_int(0).unwrap();
        let nanosec = stamp.get_uint(1).unwrap();
        let frame_id = header.get_string(1).unwrap().to_string();
        let height = row.get_uint(2).unwrap();
        let width = row.get_uint(3).unwrap();
        let encoding = row.get_string(4).unwrap().to_string();
        let is_bigendian = row.get_bool(5).unwrap();
        let step = row.get_uint(6).unwrap();
        let image_oid = row.get_string(7).unwrap().to_string();

        Image::new(
            sec,
            nanosec,
            frame_id,
            height,
            width,
            encoding,
            is_bigendian,
            step,
            image_oid,
        )
    }
}
