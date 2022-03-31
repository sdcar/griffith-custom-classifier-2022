extern crate sdcar_pcl;
extern crate sdcar_types;

use sdcar_pcl::prelude::*;
use std::time::Instant;
use chrono::{DateTime, Utc};

fn print_parquet(path: &str) {
    let iter = PointCloudXYZIntensityRowIter::new(path);
    let mut ts_last: Option<i64> = None;

    for pcxyzi in iter {
        println!(
            "pcxyzi timestamp: {} point size: {} pointxyzi[0] {:?}",
            pcxyzi.timestamp(),
            pcxyzi.point_indices_ref().len(),
            pcxyzi.point_indices_ref()[0]
        );

        match ts_last {
            Some(ts_last) => {
              if pcxyzi.timestamp().timestamp_nanos() < ts_last {
                  println!("ERROR timestamps not in sequence");
              }
            },
            None => {}
        }
        ts_last=Some(pcxyzi.timestamp().timestamp_nanos());
        // let pcxyzi_mm_start = Instant::now();
        // let pcmm = get_min_max::<XYZ, Intensity>(&pcxyzi);
        // let pcxyzi_mm_loaded = Instant::now();
        // println!(
        //     "{:?} min max {:?}",
        //     pcxyzi_mm_loaded.duration_since(pcxyzi_mm_start),
        //     pcmm
        // );
    }
}
pub fn main() {
    print_parquet("/u02/data/poc5/20210921_1st_data_collection/parquet/point_cloud2_20211028105702.parquet");
}
