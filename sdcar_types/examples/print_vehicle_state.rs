extern crate parquet;
extern crate sdcar_types;

use parquet::file::reader::{FileReader, SerializedFileReader};
use sdcar_types::msg::vehicle_state::VehicleState;
use std::fs::File;
use std::path::Path;

fn print_parquet(path: &str) {
    let file = File::open(&Path::new(path)).unwrap();
    let reader = SerializedFileReader::new(file).unwrap();
    let iter = reader.get_row_iter(None).unwrap();

    for row in iter {
        let vehicle_state = VehicleState::from(row);
        println!("{:?}", vehicle_state);
    }
}
fn main() {
    print_parquet("/u01/data/poc4/20200718/vehicle_state_20200718144724.parquet");
}
