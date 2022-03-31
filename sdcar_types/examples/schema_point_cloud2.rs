extern crate parquet;

use parquet::file::reader::{FileReader, SerializedFileReader};
use parquet::record::Row;
use parquet::schema::printer;
use std::fs::File;
use std::path::Path;

fn schema_parquet(path: &str) {
    let file = File::open(&Path::new(path)).unwrap();
    let reader = SerializedFileReader::new(file).unwrap();
    let metadata = reader.metadata();
    let schema = metadata.file_metadata().schema();

    let mut buf = Vec::new();

    printer::print_schema(&mut buf, &schema);

    let string_schema = String::from_utf8(buf).unwrap();
    println!("{}", string_schema);

    let row: Row = reader.get_row_iter(None).unwrap().next().unwrap();
    for (idx, (name, field)) in row.get_column_iter().enumerate() {
        if name != "data" {
            println!(
                "column index: {}, column name: {}, column value: {}",
                idx, name, field
            );
        } else {
            println!(
                "column index: {}, column name: {}, column value: binary",
                idx, name
            );
        }
    }
}
fn main() {
    schema_parquet("/u02/data/poc5/20210921_1st_data_collection/parquet/point_cloud2_20211016141835.parquet");
}
