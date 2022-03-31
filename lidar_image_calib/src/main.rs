extern crate nalgebra as na;
extern crate opencv;
extern crate sdcar_calib;
extern crate sdcar_pcl;
extern crate sdcar_types;
extern crate sdcar_vision;

use opencv::calib3d;
use opencv::core::*;
use opencv::highgui;
use opencv::imgcodecs;
use opencv::imgproc;
use opencv::imgproc::{get_text_size, FILLED, FONT_ITALIC, LINE_4};
use opencv::videoio::{VideoWriter};
use opencv::prelude::*;

use sdcar_pcl::prelude::*;
use sdcar_types::reader::parquet::*;
use std::path::Path;
use std::sync::{Arc, RwLock};

use std::time::{Duration, Instant};

mod fuse;

use fuse::*;
use opencv::Result;
use sdcar_calib::lidar::*;
use sdcar_vision::bounding_box::ImageBoundingBoxesXYZIntensityRef;
// use sdcar_vision::camera::*;
use sdcar_calib::CalibFrame;
use sdcar_vision::fusion::ClusterLiDARWithTmageBB;
use sdcar_vision::lidar_to_camera::LiDAR2Camera;
use sdcar_vision::object_detection_2d::*;
use sdcar_vision::stereo_calibration::StereoCalibration;
use std::collections::VecDeque;
// use sdcar_calib::twiddle::rt_3d_2d::*;

use sdcar_vision::stereo_rectify::StereoRectify;

use std::f64::consts::PI;

struct AppState {
    next_point_cloud_ref: Option<PointCloudXYZIntensityRef>,
    next_image_file_ref: Option<ImageFileRef>,
    // camera: PinholeCameraRef,
    stereo_calibration: StereoCalibration,
    lidar_to_camera: LiDAR2Camera,
    camera_image: Option<Mat>,
    clusters: ClustersRef<XYZ, Intensity>,
    dnn_object_detection: DNNObjectDetection,
    image_bounding_boxes: Option<ImageBoundingBoxesXYZIntensityRef>,
    lidar_bounding_boxes: Option<BoundingBoxesXYZIntensityRef>,
    calib_frames: VecDeque<CalibFrame<XYZ, Intensity>>,
    // generation: Option<Generation<XYZ,Intensity>>,
    draw_target_cloud: bool,
    draw_proj_cloud: bool,
    draw_image_bb_cloud: bool,
}

impl AppState {
    fn new(
        next_point_cloud_ref: Option<PointCloudXYZIntensityRef>,
        next_image_file_ref: Option<ImageFileRef>,
        // camera: PinholeCameraRef,
        stereo_calibration: StereoCalibration,
        lidar_to_camera: LiDAR2Camera,
        camera_image: Option<Mat>,
        clusters: ClustersRef<XYZ, Intensity>,
        dnn_object_detection: DNNObjectDetection,
        image_bounding_boxes: Option<ImageBoundingBoxesXYZIntensityRef>,
        lidar_bounding_boxes: Option<BoundingBoxesXYZIntensityRef>,
        calib_frames: VecDeque<CalibFrame<XYZ, Intensity>>,
    ) -> Self {
        Self {
            next_point_cloud_ref,
            next_image_file_ref,
            // camera,
            stereo_calibration,
            lidar_to_camera,
            camera_image,
            clusters,
            dnn_object_detection,
            image_bounding_boxes,
            lidar_bounding_boxes,
            calib_frames,
            // generation:None,
            draw_target_cloud: false,
            draw_proj_cloud: false,
            draw_image_bb_cloud: true,
        }
    }
    fn next_point_cloud(&mut self, next_point_cloud_ref: PointCloudXYZIntensityRef) {
        self.next_point_cloud_ref = Some(next_point_cloud_ref);
    }
    fn next_image_file(&mut self, next_image_file_ref: ImageFileRef) {
        self.next_image_file_ref = Some(next_image_file_ref);
    }
    fn image_file_name(&self) -> String {
        let image_file = self.next_image_file_ref.clone().expect("no image file");
        image_file.file_name()
    }
    fn point_cloud_ref(&self) -> PointCloudXYZIntensityRef {
        self.next_point_cloud_ref.clone().expect("no point cloud")
    }
    fn state_image(&self) -> Result<Mat> {
        let calib_frame = self.calib_frames[self.calib_frames.len() - 1].clone();
        let mut vis_img = Mat::default();
        calib_frame.camera_img.copy_to(&mut vis_img)?;
        let target_box_id: u32 = if let Some(target_bb) = calib_frame.target_image_bb.clone() {
            target_bb.box_id
        } else {
            99999 as u32
        };
        // let roi = Rect2i::new(10,10,20,15);
        // let color = Scalar::new(256.0, 0.0, 0.0, 0.0);
        // imgproc::rectangle(&mut vis_img, roi, color, 1, LINE_4, 0)?;
        if let Some(bbes) = calib_frame.image_bounding_boxes.clone() {
            for bb in bbes.read().unwrap().iter() {
                let top = bb.roi.y;
                let left = bb.roi.x;

                let color = if bb.box_id == target_box_id {
                    Scalar::new(128.0, 128.0, 256.0, 0.)
                } else {
                    Scalar::new(255.0, 128.0, 128.0, 0.)
                };

                imgproc::rectangle(&mut vis_img, bb.roi, color, 1, LINE_4, 0)?;

                let label = format!("{} {} {:3.2}", bb.box_id, bb.class, bb.confidence);

                // println!("bb {:?}", bb);

                // display label at the top of the bounding box
                let mut base_line = 0;
                let label_size = get_text_size(&label, FONT_ITALIC, 0.5, 1, &mut base_line)?;
                let top = std::cmp::max(top, label_size.height);
                let y = top - (1.25 * label_size.height as f32).round() as i32;
                let rec = Rect2i::new(
                    left,
                    y,
                    (1.05 * label_size.width as f32).round() as i32,
                    base_line + (1.25 * label_size.height as f32).round() as i32,
                );
                imgproc::rectangle(&mut vis_img, rec, color, FILLED, LINE_4, 0)?;
                imgproc::put_text(
                    &mut vis_img,
                    &label,
                    Point2i::new(left, top),
                    FONT_ITALIC,
                    0.5,
                    Scalar::default(),
                    1,
                    LINE_4,
                    false,
                )?;
            }
        }
        if self.draw_target_cloud {
            if let Some(lidar_points) = calib_frame.target_lidar_points {
                // convert from lidar points into OpenCV Vector Points
                let mut object_points = Vector::<Point3d>::new();
                let mut image_points = Vector::<Point2f>::new();
                for lp in lidar_points.iter() {
                    let (x, y, z) = lp.clone().point().tuple();
                    object_points.push(Point3d::new(x.into(), y.into(), z.into()));

                    let p_2d = self
                        .lidar_to_camera
                        .project_lidar_to_image(na::Point3::new(x.into(), y.into(), z.into()))
                        .unwrap();
                    image_points.push(Point2f::new(p_2d[0] as f32, p_2d[1] as f32));
                }

                let color = Scalar::new(0.0, 256.0, 128.0, 0.0);
                let mut num_points = 0;
                // print!("projected:");
                for p in image_points.iter() {
                    let p = Point2i::new(p.x as i32, p.y as i32);
                    // print!(" {:?}", p);
                    num_points += 1;
                    // if num_points % 4 == 0 {
                    //     println!("");
                    // }
                    imgproc::circle(&mut vis_img, p, 3, color, 2, LINE_4, 0)?;
                }
                // println!("");
            }
        }
        if self.draw_proj_cloud {
            if let Some(point_cloud_ref) = calib_frame.lidar_points {
                // convert from lidar points into camera OpenCV Vector Points
                let mut object_points = Vector::<Point3d>::new();
                let mut image_points = Vector::<Point2f>::new();

                let fov_abs = (45.0 * PI / 180.0) as f64;
                let mut op_min = (f64::MAX, f64::MAX, f64::MAX);
                let mut op_max = (f64::MIN, f64::MIN, f64::MIN);
                for lp in point_cloud_ref.point_indices_ref().iter() {
                    let (x, y, z) = lp.clone().point().tuple();
                    let x = x as f64;
                    let y = y as f64;
                    let z = z as f64;
                    if x > 0.0 && y.atan2(x).abs() < fov_abs
                    // && z> -2.0 && z <1.0
                    {
                        object_points.push(Point3d::new(x as f64, y as f64, z as f64));
                        if x < op_min.0 {
                            op_min.0 = x;
                        }
                        if x > op_max.0 {
                            op_max.0 = x;
                        }
                        if y < op_min.1 {
                            op_min.1 = y;
                        }
                        if y > op_max.1 {
                            op_max.1 = y;
                        }
                        if z < op_min.2 {
                            op_min.2 = z;
                        }
                        if z > op_max.2 {
                            op_max.2 = z;
                        }
                        let p_2d = self
                            .lidar_to_camera
                            .project_lidar_to_image(na::Point3::new(x.into(), y.into(), z.into()))
                            .unwrap();
                        image_points.push(Point2f::new(p_2d[0] as f32, p_2d[1] as f32));
                    }
                }

                Self::draw_image_points(
                    &mut vis_img,
                    &image_points,
                    &object_points,
                    op_min,
                    op_max,
                );
            }
        }

        if self.draw_image_bb_cloud {
            if let Some(bbes) = calib_frame.image_bounding_boxes.clone() {
                let mut object_points = Vector::<Point3d>::new();
                let mut image_points = Vector::<Point2f>::new();

                let fov_abs = (45.0 * PI / 180.0) as f64;
                let mut op_min = (f64::MAX, f64::MAX, f64::MAX);
                let mut op_max = (f64::MIN, f64::MIN, f64::MIN);

                for image_bb in bbes.read().unwrap().iter() {
                    if let Some(lidar_points) = &image_bb.lidar_points {
                        for lp in lidar_points.into_iter() {
                            let (x, y, z) = lp.clone().point().tuple();
                            let x = x as f64;
                            let y = y as f64;
                            let z = z as f64;
                            if x > 0.0 && y.atan2(x).abs() < fov_abs
                            // && z> -2.0 && z <1.0
                            {
                                object_points.push(Point3d::new(x as f64, y as f64, z as f64));
                                if x < op_min.0 {
                                    op_min.0 = x;
                                }
                                if x > op_max.0 {
                                    op_max.0 = x;
                                }
                                if y < op_min.1 {
                                    op_min.1 = y;
                                }
                                if y > op_max.1 {
                                    op_max.1 = y;
                                }
                                if z < op_min.2 {
                                    op_min.2 = z;
                                }
                                if z > op_max.2 {
                                    op_max.2 = z;
                                }
                                let p_2d = self
                                    .lidar_to_camera
                                    .project_lidar_to_image(na::Point3::new(
                                        x.into(),
                                        y.into(),
                                        z.into(),
                                    ))
                                    .unwrap();
                                image_points.push(Point2f::new(p_2d[0] as f32, p_2d[1] as f32));
                            }
                        }
                    }
                }
                Self::draw_image_points(
                    &mut vis_img,
                    &image_points,
                    &object_points,
                    op_min,
                    op_max,
                );
            }
        }

        Ok(vis_img)
    }

    fn draw_image_points(
        vis_img: &mut Mat,
        image_points: &Vector<Point2f>,
        object_points: &Vector<Point3d>,
        op_min: (f64, f64, f64),
        op_max: (f64, f64, f64),
    ) {
        let mut i = 0;
        let x_color_factor = 256.0 / (op_max.0 - op_min.0);
        let y_color_factor = 256.0 / (op_max.1 - op_min.0);
        let z_color_factor = 256.0 / (op_max.2 - op_min.0);
        for p in image_points.iter() {
            let p = Point2i::new(p.x as i32, p.y as i32);
            let op = object_points.get(i).unwrap();
            i = i + 1;
            // let color = Scalar::new(128.0+op.y as f64, 128.0+op.x as f64, 128.0+op.z as f64, 0.0);
            let color = Scalar::new(
                z_color_factor * op.z as f64 + 32.0,
                x_color_factor * op.x as f64 + 32.0,
                y_color_factor * op.y.abs() as f64,
                0.0,
            );
            imgproc::circle(vis_img, p, 2, color, 2, LINE_4, 0);
        }
    }
}

fn string_to_static_str(s: String) -> &'static str {
    Box::leak(s.into_boxed_str())
}

fn main() -> Result<()> {
    println!("OpenCV version: {:?}", CV_VERSION);

    // let path = "/u02/data/poc5/20210921_1st_data_collection";
    // let path = "/Users/nick/data/poc5/20210921_1st_data_collection";
    let path = "/u02/data/poc5/20211214_2nd_data_collection";
    // let run = "20211028165901";
    let run = "20211216145545";
    let pc2_file = string_to_static_str(format!("{}/parquet/point_cloud2_{}.parquet", path, run));
    let image_file = string_to_static_str(format!("{}/parquet/left_image_{}.parquet", path, run));
    let image_data_path = string_to_static_str(format!("{}/images/rectified/", path));

    println!("pc2_file: {}", pc2_file);
    println!("image_file: {}", image_file);
    println!("image_data_path: {}", image_data_path);

    let point_cloud2_row_iter = PointCloud2ParquetRowIter::<'static>::new(pc2_file);
    let image_row_iter = ImageParquetRowIter::<'static>::new(image_file);
    let mut point_cloud_image_iter = PointCloudXYZIntensityImageRowIter::new(
        point_cloud2_row_iter,
        image_row_iter,
        image_data_path,
    );

    let clusters: ClustersRef<XYZ, Intensity> =
        ClustersRef::<XYZ, Intensity>::new(Clusters::<XYZ, Intensity>::new());

    // setup 2d image object detection
    let confidence_threshold = 0.4;
    let nms_threshold = 0.5;
    let classes_file =
        "/home/nick/feature-extraction-and-map-poc5/data_extract_viz/yolo/cfg/coco.names";
        // "/Users/nick/feature-extraction-and-map-poc5/data_extract_viz/yolo/cfg/coco.names";
    let cfg_file =
        "/home/nick/feature-extraction-and-map-poc5/data_extract_viz/yolo/cfg/yolov4.cfg";
        // "/Users/nick/feature-extraction-and-map-poc5/data_extract_viz/yolo/cfg/yolov4.cfg";
    let darknet_model =
        "/home/nick/feature-extraction-and-map-poc5/data_extract_viz/yolo/yolov4.weights";
        // "/Users/nick/feature-extraction-and-map-poc5/data_extract_viz/yolo/yolov4.weights";
    let dnn_object_detection = DNNObjectDetection::new(
        confidence_threshold,
        nms_threshold,
        classes_file,
        cfg_file,
        darknet_model,
    );

    let image_size = Size::new(2048, 1536);
    println!("image_size: {:?}", &image_size);
    let calibration_path =
        "/home/nick/feature-extraction-and-map-poc5/sdcar_vision/calibration/calibration.yaml";
        // "/Users/nick/feature-extraction-and-map-poc5/sdcar_vision/calibration/calibration.yaml";
    println!("calibration_path: {}", &calibration_path);
    let stereo_calibration = StereoCalibration::new(calibration_path);
    println!("stereo_calibration: {:#?}", &stereo_calibration);
    let stereo_rectify = StereoRectify::new(stereo_calibration.clone(), image_size.clone());
    println!("stereo_rectify: {:#?}", &stereo_rectify);

    let lidar_to_camera = LiDAR2Camera::from_left_camera(stereo_calibration.clone());
    println!("lidar_to_camera: {:#?}", &lidar_to_camera);

    let cluster_lidar_with_image = ClusterLiDARWithTmageBB::new(lidar_to_camera.clone());

    let calib_frames = VecDeque::<CalibFrame<XYZ, Intensity>>::new();

    let mut app_state = AppState::new(
        None,
        None,
        // camera,
        stereo_calibration,
        lidar_to_camera,
        None,
        clusters,
        dnn_object_detection,
        None,
        None,
        calib_frames,
    );

    let four_cc = VideoWriter::fourcc('x' as i8, '2' as i8, '6' as i8, '4' as i8).unwrap();
    let mut video_writer = VideoWriter::new("image_lidar.mkv", four_cc, 50.0, image_size, true).unwrap();

    let mut keep_iterating = true;
    while keep_iterating {
        if let Some(data) = point_cloud_image_iter.next() {
            match data {
                DataKind::PointCloud(pc) => {
                    app_state.next_point_cloud(pc);
                    let (_inlier_ref, outlier_ref, clusters_ref, bounding_box_ref) =
                        do_point_cloud(app_state.point_cloud_ref());
                    app_state.lidar_bounding_boxes = Some(bounding_box_ref);
                    app_state.clusters = clusters_ref;
                    // print_lidar_bb(app_state.lidar_bounding_boxes.clone());
                    show_lidar_top_view(outlier_ref, app_state.lidar_bounding_boxes.clone(), false)
                        .unwrap();
                }
                DataKind::ImageFile(image_file) => {
                    app_state.next_image_file(image_file.clone());
                    // println!("app_state.image_file_name(): {}", &app_state.image_file_name());
                    let image =
                        imgcodecs::imread(&app_state.image_file_name(), imgcodecs::IMREAD_COLOR)
                            .unwrap();
                    // let image = stereo_rectify.rectify_left(image);
                    app_state.camera_image = Some(image.clone());
                    // highgui::named_window("daheng_left", 0).unwrap();
                    // highgui::imshow("daheng_left", &image).unwrap();
                    // highgui::wait_key(15).unwrap();
                    let vis = false;
                    // detect_objects(&image, confidence_threshold, nms_threshold, classes_file, cfg_file, darknet_model, vis).unwrap();
                    let bounding_boxes = app_state
                        .dnn_object_detection
                        .detect_objects(&image, vis, false, false)
                        .unwrap();
                    app_state.image_bounding_boxes = Some(Arc::new(RwLock::new(bounding_boxes)));
                    // print_image_bb(app_state.image_bounding_boxes.clone());
                }
                _ => {}
            }
            if app_state.camera_image.clone().is_some() && app_state.next_point_cloud_ref.is_some()
            {
                cluster_lidar_with_image.cluster(
                    app_state.point_cloud_ref().clone(),
                    app_state.image_bounding_boxes.clone().unwrap().clone(),
                    0.1,
                );

                let calib_frame = fuse_image_lidar(
                    app_state.camera_image.clone().unwrap(),
                    Some(app_state.point_cloud_ref().clone()),
                    app_state.image_bounding_boxes.clone(),
                    app_state.lidar_bounding_boxes.clone(),
                    app_state.clusters.clone(),
                );
                app_state.calib_frames.push_back(calib_frame);
                while app_state.calib_frames.len() > 2 {
                    app_state.calib_frames.pop_front();
                }
                let vis_img = app_state.state_image()?;

                video_writer.write(&vis_img);

                let winname = "App State";
                highgui::named_window(winname, highgui::WINDOW_AUTOSIZE)?;
                highgui::imshow(winname, &vis_img)?;
                // println!("{:?}", image_frame);
            }
            let wait_key = highgui::wait_key(5).unwrap();
            if wait_key == 113 {
                keep_iterating = false;
            }
        } else {
            keep_iterating = false;
        }
    }
    video_writer.release();
    Ok(())
}

fn print_image_bb(bbs_optional: Option<ImageBoundingBoxesXYZIntensityRef>) {
    if let Some(bbs) = bbs_optional {
        println!("image");
        for bb in bbs.read().unwrap().iter() {
            println!("{:?}", bb);
        }
    }
}

fn print_lidar_bb(bbs_optional: Option<BoundingBoxesXYZIntensityRef>) {
    if let Some(bbs) = bbs_optional {
        println!("lidar");
        for bb in bbs.iter().cloned() {
            println!("{:?}", bb);
        }
    }
}
