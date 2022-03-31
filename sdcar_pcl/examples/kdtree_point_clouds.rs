extern crate nalgebra;
extern crate parquet;
extern crate sdcar_pcl;
extern crate sdcar_types;

use nalgebra::Vector3;
use parquet::file::reader::{FileReader, SerializedFileReader};
use sdcar_pcl::prelude::*;
use sdcar_types::msg::point_cloud2::PointCloud2;
use std::fs::File;
use std::path::Path;
use std::sync::Arc;
use std::time::Instant;

fn kdtree_parquet(path: &str) {
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
        let dsxyzi = voxel_grid::filter_points(&pcxyzi, leaf_count);
        let pcxyzi_ds_loaded = Instant::now();
        println!(
            "{:?} down_sample {:?}",
            pcxyzi_ds_loaded.duration_since(pcxyzi_ds_start),
            dsxyzi.num_points()
        );

        let cropped_start = Instant::now();
        let min_vec = MinVec::new(-120.0, -5.0, -4.75);
        let max_vec = MaxVec::new(120.0, 5.0, 2.0);
        let cropped = crop_box::point_indices_bounding_box(
            dsxyzi.point_indices_ref(),
            min_vec,
            max_vec,
            false,
        );
        let cropped_end = Instant::now();
        println!(
            "{:?} cropped {:?}",
            cropped_end.duration_since(cropped_start),
            cropped.len()
        );

        let ref_in_start = Instant::now();
        // let (inlier_ref, outlier_ref) = segment_planes_ransac(&dsxyzi, 10, 0.5);
        let (inlier_ref, outlier_ref) =
            segment_planes_ransac(&PointCloudXYZIntensity::from(cropped.clone()), 10, 0.5);
        let ref_in_loaded = Instant::now();
        println!(
            "{:?} cloud point: {} split inliers: {} outliers: {}",
            ref_in_loaded.duration_since(ref_in_start),
            cropped.len(),
            inlier_ref.num_points(),
            outlier_ref.num_points()
        );

        let kdtree_start = Instant::now();
        let kdtree_ref = Arc::new(kdtree::KdTreePointXYZIntensity::from(
            outlier_ref.point_indices_ref(),
        ));
        let kdtree_loaded = Instant::now();
        println!(
            "{:?} kdtree loaded",
            kdtree_loaded.duration_since(kdtree_start)
        );
        // kdtree_ref.print_tree();

        let clusters_start = Instant::now();
        let cluster_extract = extract_clusters::ClustersExtractXYZIntensity::new(
            outlier_ref,
            10.0,
            20,
            200,
            kdtree_ref,
        );
        let clusters = cluster_extract.extract();
        let clusters_loaded = Instant::now();
        println!(
            "{:?} {} clusters extracted",
            clusters_loaded.duration_since(clusters_start),
            clusters.len()
        );

        for cluster in clusters {
            let bb = BoundingBoxXYZIntensity::from(cluster);
            println!("{:?}", bb);
        }
    }
}

pub fn main() {
    kdtree_parquet("/u01/data/poc4/20200718/point_cloud2_20200718144724.parquet");
}
