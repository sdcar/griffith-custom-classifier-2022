extern crate parquet;
extern crate sdcar_types;

use sdcar_types::msg::*;
use sdcar_types::reader::combined::*;
use sdcar_types::reader::parquet::*;

fn print_parquet(path1: &'static str, path2: &'static str) {
    let image_iter = ImageParquetRowIter::new(&path1);
    let lidar_iter = PointCloud2ParquetRowIter::new(&path2);

    let reader =
        Reader2::<PointCloud2ParquetRowIter, ImageParquetRowIter>::new(lidar_iter, image_iter);

    let mut pc2_count = 0;
    let mut image_count = 0;
    for msg in reader {
        print!("{:?}", msg.timestamp());
        match msg.msg_kind() {
            MsgKind::PointCloud2(point_cloud2_ref) => {
                println!(
                    " PointCloud2 height: {} width: {}",
                    point_cloud2_ref.height, point_cloud2_ref.width
                );
                pc2_count += 1;
            }
            MsgKind::Image(image_ref) => {
                println!(
                    " Image height: {} width: {}",
                    image_ref.height, image_ref.width
                );
                image_count += 1
            }
            _ => println!("not implemented"),
        };
    }
    println!("image_count: {}", image_count);
    println!("pc2_count: {}", pc2_count);
}
fn main() {
    let path1 =
        "/u02/data/poc5/20210921_1st_data_collection/parquet/left_image_20211028165901.parquet";
    let path2 =
        "/u02/data/poc5/20210921_1st_data_collection/parquet/point_cloud2_20211028165901.parquet";

    print_parquet(path1, path2);
}
