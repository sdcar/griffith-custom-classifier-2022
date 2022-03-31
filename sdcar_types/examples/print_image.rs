extern crate parquet;
extern crate sdcar_types;

use sdcar_types::reader::parquet::*;

fn print_parquet(path: &str) {
    let iter = ImageParquetRowIter::new(&path);

    for image in iter {
        println!("{:?}", image)
    }
}
fn main() {
    print_parquet("/u02/data/poc5/20210921_1st_data_collection/parquet/left_image_20211016141835.parquet");
}
