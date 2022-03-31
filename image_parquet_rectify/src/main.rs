extern crate opencv;
extern crate sdcar_types;
extern crate sdcar_vision;

use std::fs;
use std::path::Path;

use opencv::core::{Size,Vector};
use opencv::imgcodecs;
use opencv::highgui;
use opencv::imgcodecs::IMWRITE_WEBP_QUALITY;
use opencv::imgproc::{get_text_size, FILLED, FONT_ITALIC, LINE_4};
use opencv::prelude::*;

use sdcar_types::reader::parquet::{HasRowIter, ImageParquetRowIter};

use sdcar_vision::stereo_calibration::StereoCalibration;
use sdcar_vision::stereo_rectify::StereoRectify;

fn string_to_static_str(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

fn main() {
    println!("OpenCV version: {:?}", opencv::core::CV_VERSION);

    // let path = Path::new("/u02/data/poc5/20210921_1st_data_collection");
    // let path = Path::new("/Users/nick/data/poc5/20210921_1st_data_collection");
    let path = Path::new("/u02/data/poc5/20211214_2nd_data_collection");
    // let run = "20211028165901";
    let run = "20211216145545";
    let left_image_file_str = string_to_static_str(format!(
        "{}/parquet/left_image_{}.parquet",
        &path.display(),
        run
    ));
    let left_image_file = Path::new(&left_image_file_str);
    let right_image_file_str = string_to_static_str(format!(
        "{}/parquet/right_image_{}.parquet",
        &path.display(),
        run
    ));
    let right_image_file = Path::new(&right_image_file_str);
    let image_data_path_str = string_to_static_str(format!("{}/images", &path.display()));
    let image_data_path = Path::new(&image_data_path_str);
    let rectified_image_path_str =
        string_to_static_str(format!("{}/images/rectified", &path.display()));
    let rectified_image_path = Path::new(&rectified_image_path_str);

    println!("left_image_file: {}", &left_image_file.display());
    println!("right_image_file: {}", &right_image_file.display());
    println!("image_data_path: {}", &image_data_path.display());
    if !Path::new(&rectified_image_path).exists() {
        println!(
            "creating rectified_image_path: {}",
            &rectified_image_path.display()
        );
        fs::create_dir_all(&rectified_image_path).unwrap();
    } else {
        println!("rectified_image_path: {}", &rectified_image_path.display());
        if !rectified_image_path.is_dir() {
            println!("rectified_image_path is not a directory - exiting!");
            return;
        }
    }

    let left_image_row_iter = ImageParquetRowIter::new(left_image_file_str);
    let right_image_row_iter = ImageParquetRowIter::new(right_image_file_str);

    let image_size = Size::new(2048, 1536);
    println!("image_size: {:?}", &image_size);
    let calibration_path = Path::new(
        "/home/nick/feature-extraction-and-map-poc5/sdcar_vision/calibration/calibration.yaml",
        // "/Users/nick/feature-extraction-and-map-poc5/sdcar_vision/calibration/calibration.yaml",
    );
    println!("calibration_path: {}", &calibration_path.display());
    let stereo_calibration = StereoCalibration::new(&calibration_path.to_string_lossy());
    println!("stereo_calibration: {:#?}", &stereo_calibration);
    let stereo_rectify = StereoRectify::new(stereo_calibration.clone(), image_size.clone());
    println!("stereo_rectify: {:#?}", &stereo_rectify);

    let mut webp_params = Vector::<i32>::default();
    webp_params.push(IMWRITE_WEBP_QUALITY);
    webp_params.push(70);

    let rectify = |oid, camera: &str, vis: bool, wait: bool| {
        let image_file_name = format!("{}/{}.webp", image_data_path_str, oid);
        println!("{}", &image_file_name);
        let image = imgcodecs::imread(&image_file_name, imgcodecs::IMREAD_COLOR).unwrap();

        if vis {
            let mut vis_img = Mat::default();
            image.copy_to(&mut vis_img).unwrap();

            let winname = camera.to_owned() + " image";
            highgui::named_window(winname.as_str(), highgui::WINDOW_AUTOSIZE).unwrap();
            highgui::imshow(winname.as_str(), &vis_img).unwrap();
            if wait {
                highgui::wait_key(0).unwrap();
             } else {
                highgui::wait_key(5).unwrap();
             }
        }
        let image = match camera {
            "left" => stereo_rectify.rectify_left(image),
            "right" => stereo_rectify.rectify_right(image),
            _ => {
                println!("error: match camera defaulted!");
                Mat::default()},
        };
        let rectified_file_name = format!("{}/{}.webp", rectified_image_path_str, oid);
        println!("{}", &rectified_file_name);
        if vis {
            let mut vis_img_rectified = Mat::default();
            image.copy_to(&mut vis_img_rectified).unwrap();

            let winname = camera.to_owned() + " image rectified ";
            highgui::named_window(winname.as_str(), highgui::WINDOW_AUTOSIZE).unwrap();
            highgui::imshow(winname.as_str(), &vis_img_rectified).unwrap();
            if wait {
                highgui::wait_key(0).unwrap();
             } else {
                highgui::wait_key(5).unwrap();
             }
        }
        imgcodecs::imwrite(&rectified_file_name, &image, &webp_params).unwrap();
    };

    let vis:bool = true;
    let wait:bool = false;
    left_image_row_iter.for_each(|image| rectify(image.image_oid.clone(), "left", vis, wait));
    right_image_row_iter.for_each(|image| rectify(image.image_oid.clone(), "right", vis, wait));
}
