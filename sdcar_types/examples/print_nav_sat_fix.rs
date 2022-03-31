extern crate parquet;
extern crate sdcar_types;

use parquet::file::reader::{FileReader, SerializedFileReader};
use sdcar_types::msg::nav_sat_fix::NavSatFix;
use std::fs::File;
use std::path::Path;

fn print_parquet(path: &str) {
    let file = File::open(&Path::new(path)).unwrap();
    let reader = SerializedFileReader::new(file).unwrap();
    let iter = reader.get_row_iter(None).unwrap();

    for row in iter {
        let gps = NavSatFix::from(row);
        println!("{:?}", gps);
    }
}
fn main() {
    print_parquet("/u01/data/poc4/20200718/nav_sat_fix_20200718144724.parquet");
}
