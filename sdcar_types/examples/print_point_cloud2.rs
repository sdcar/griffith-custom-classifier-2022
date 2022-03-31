extern crate parquet;
extern crate sdcar_types;

use sdcar_types::reader::parquet::*;

fn print_parquet(path: &str) {
    let iter = PointCloud2ParquetRowIter::new(&path);

    for pc in iter {
        println!(
            "{} {} {} {} {} {} {:?} {} {} {} data.len() = {} {}",
            pc.stamp_sec,
            pc.stamp_nanosec,
            pc.timestamp,
            pc.frame_id,
            pc.height,
            pc.width,
            pc.fields,
            pc.is_bigendian,
            pc.point_step,
            pc.row_step,
            pc.data.len(),
            pc.is_dense
        );
    }
}
fn main() {
    print_parquet("/u02/data/poc5/20210921_1st_data_collection/parquet/point_cloud2_20211016141835.parquet");
}
