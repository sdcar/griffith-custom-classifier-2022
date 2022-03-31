extern crate nalgebra as na;
extern crate parquet;
extern crate sdcar_pcl;
extern crate sdcar_types;

use parquet::file::reader::{FileReader, SerializedFileReader};
use sdcar_pcl::prelude::*;
use sdcar_types::msg::point_cloud2::PointCloud2;
use std::fs::File;
use std::path::Path;
use std::time::Instant;

use na::Vector3;

fn downsample_parquet(path: &str) {
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

        let pcxyzi_ds_start = Instant::now();
        let leaf_count = Vector3::<usize>::new(1000, 1000, 100);
        let dsxyzi = filter_points(&pcxyzi, leaf_count);
        let pcxyzi_ds_loaded = Instant::now();
        println!(
            "{:?} down_sample {:?}",
            pcxyzi_ds_loaded.duration_since(pcxyzi_ds_start),
            dsxyzi.num_points()
        );
    }
}
pub fn main() {
    downsample_parquet("/u01/data/poc4/20200718/point_cloud2_20200718144724.parquet");
}
