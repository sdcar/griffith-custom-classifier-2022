extern crate sdcar_pcl;
extern crate sdcar_types;

use sdcar_pcl::prelude::*;
use sdcar_types::reader::parquet::*;

fn print_parquet(pc2_file: &'static str, image_file: &'static str, image_data_path: &'static str) {
    let point_cloud2_row_iter = PointCloud2ParquetRowIter::<'static>::new(pc2_file);
    let image_row_iter = ImageParquetRowIter::<'static>::new(image_file);
    let iter = PointCloudXYZIntensityImageRowIter::new(
        point_cloud2_row_iter,
        image_row_iter,
        image_data_path,
    );
    let mut pc_count = 0;
    let mut if_count = 0;
    for data in iter {
        match data {
            DataKind::PointCloud(pcxyzi) => {
                println!(
                    "pcxyzi timestamp: {} point size: {} pointxyzi[0] {:?}",
                    pcxyzi.timestamp(),
                    pcxyzi.point_indices_ref().len(),
                    pcxyzi.point_indices_ref()[0]
                );
                pc_count += 1;
            }

            DataKind::Point(_) => println!("Error: point data shouldnt be here"),
            DataKind::BoundingBox(_) => println!("Error: bound box data shouldnt be here"),
            DataKind::ImageFile(image_file_ref) => {
                println!(
                    "image  timestamp: {} filename: {}",
                    image_file_ref.timestamp(),
                    image_file_ref.file_name()
                );
                if_count += 1;
            }
        }
    }
    println!("point cloud count: {}", pc_count);
    println!("image file count: {}", if_count);
}
pub fn main() {
    let pc2_file = "/u02/data/poc5/20210921_1st_data_collection/parquet/point_cloud2_20211028165901.parquet";
    let image_file = "/u02/data/poc5/20210921_1st_data_collection/parquet/left_image_20211028165901.parquet";
    let image_data_path = "/u02/data/poc5/20210921_1st_data_collection/images";
    print_parquet(pc2_file, image_file, image_data_path);
}
