extern crate parquet;
extern crate sdcar_pcl;
extern crate sdcar_types;

use parquet::file::reader::{FileReader, SerializedFileReader};
use sdcar_pcl::prelude::*;
use sdcar_types::msg::point_cloud2::PointCloud2;
use std::fs::File;
use std::path::Path;
use std::time::Instant;

fn print_parquet(path: &str) {
    let file = File::open(&Path::new(path)).unwrap();
    let reader = SerializedFileReader::new(file).unwrap();
    let iter = reader.get_row_iter(None).unwrap();

    for row in iter {
        let row_start = Instant::now();
        let pc = PointCloud2::from(row);
        let row_loaded = Instant::now();
        println!(
            "{:?} {} {} {} {} {} {} {:?} {} {} {} data.len() = {} {}",
            row_loaded.duration_since(row_start),
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

        let pcxyzi_start = Instant::now();
        let pcxyzi = PointCloudXYZIntensity::from(pc);
        let pcxyzi_loaded = Instant::now();
        println!(
            "{:?} pcxyzi timestamp: {} point size: {} pointxyzi[0] {:?}",
            pcxyzi_loaded.duration_since(pcxyzi_start),
            pcxyzi.timestamp(),
            pcxyzi.point_indices_ref().len(),
            pcxyzi.point_indices_ref()[0]
        );

        let pcxyzi_mm_start = Instant::now();
        let pcmm = get_min_max::<XYZ, Intensity>(&pcxyzi);
        let pcxyzi_mm_loaded = Instant::now();
        println!(
            "{:?} min max {:?}",
            pcxyzi_mm_loaded.duration_since(pcxyzi_mm_start),
            pcmm
        );
    }
}
pub fn main() {
    print_parquet("/u01/data/poc4/20200718/point_cloud2_20200718144724.parquet");
}
