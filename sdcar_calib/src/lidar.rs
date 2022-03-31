extern crate nalgebra as na;
extern crate opencv;
extern crate sdcar_pcl;

use opencv::core::{self, GpuMat, Point2i, Rect2i, Scalar, Size, CV_8UC3};
use opencv::highgui;
use opencv::imgproc;
use opencv::imgproc::{get_text_size, FILLED, FONT_ITALIC, LINE_4};
use opencv::prelude::*;
use opencv::Result;

use na::{Matrix4, Point2, Point3, Translation2, Vector3};
use sdcar_pcl::prelude::*;
use std::sync::Arc;

pub fn do_point_cloud(
    point_cloud_ref: PointCloudXYZIntensityRef,
) -> (
    PointCloudXYZIntensityRef,
    PointCloudXYZIntensityRef,
    ClustersRef<XYZ, Intensity>,
    BoundingBoxesXYZIntensityRef,
) {
    let leaf_count = Vector3::<usize>::new(2000, 1200, 800);
    let down_sampled = voxel_grid::filter_points(&point_cloud_ref, leaf_count);
    let cropped = crop_box::point_indices_bounding_box(
        down_sampled.point_indices_ref(),
        MinVec::new(-50.0, -12.0, -4.75),
        MaxVec::new(50.0, 12.0, 3.0),
        false,
    );

    // remove vehicle roof points
    let cropped = crop_box::point_indices_bounding_box(
        cropped,
        MinVec::new(-2.5, -0.75, -1.0),
        MaxVec::new(1.0, 0.75, 0.2),
        true,
    );

    let (inlier_ref, outlier_ref) =
        segment_planes_ransac(&PointCloudXYZIntensity::from(cropped), 50, 0.35);

    let kdtree_ref = Arc::new(kdtree::KdTreePointXYZIntensity::from(
        outlier_ref.point_indices_ref(),
    ));
    let cluster_extract = extract_clusters::ClustersExtractXYZIntensity::new(
        outlier_ref.clone(),
        1.25,
        7,
        150,
        kdtree_ref,
    );
    let clusters = cluster_extract.extract();

    let mut bounding_boxes = BoundingBoxes::<XYZ, Intensity>::new();
    for i in 0..clusters.len() {
        let cluster = clusters[i].clone();
        let mut bb = BoundingBoxXYZIntensity::from(cluster);

        // filter out clusters that cant be cars
        let (dx, dy, dz) = bb.dxyz();
        let z_min = bb.point_min().point().z();
        let z_max = bb.point_max().point().z();
        if z_min < -4.0
            || z_max > 1.0
            || dz < 0.2
            || dz > 4.0
            || dx < 0.3
            || dx > 6.0
            || dy < 0.3
            || dy > 6.0
        {
            let max_p = bb.point_max().point().tuple();
            if max_p.0 > 0.0 && max_p.1 < 8.0 {
                println!("- {:?} {:?}", bb.dxyz(), bb);
            }
            continue;
        }

        bb.box_id = Some(bounding_boxes.len() as u32);
        bb.cluster_id = Some(i as u32);
        println!("+ {:?} {:?}", bb.dxyz(), bb);

        let bb_ref = Arc::new(bb);
        bounding_boxes.push(bb_ref);
    }

    let clusters_ref: ClustersRef<XYZ, Intensity> = Arc::new(clusters);
    let bounding_boxes_ref = Arc::new(bounding_boxes);
    (inlier_ref, outlier_ref, clusters_ref, bounding_boxes_ref)
}

const XC: i32 = 50; // center
const YC: i32 = 12;

struct TopView {
    world_size: Size,
    image_size: Size,
}

impl TopView {
    pub fn new() -> Self {
        let world_size: Size = Size::new(YC * 2, XC * 2);
        let image_size: Size = Size::new(YC * 10, XC * 10);
        Self {
            world_size,
            image_size,
        }
    }
    // convert lidar world point in top down view image point
    pub fn image_point_top(&self, xw: f32, yw: f32, zw: f32) -> (f32, f32) {
        let y = (-xw * (self.image_size.height / self.world_size.height) as f32)
            + self.image_size.height as f32 / 2.0;
        let x = (-yw * (self.image_size.width / self.world_size.width) as f32)
            + self.image_size.width as f32 / 2.0;
        (x, y)
    }
}

pub fn show_lidar_top_view(
    point_cloud_ref: PointCloudXYZIntensityRef,
    bounding_boxes: Option<BoundingBoxesXYZIntensityRef>,
    wait: bool,
) -> Result<()> {
    let tv = TopView::new();

    let mut top_view_img =
        Mat::new_size_with_default(tv.image_size, CV_8UC3, Scalar::new(0.0, 0.0, 0.0, 0.0))?;
    imgproc::rectangle(
        &mut top_view_img,
        Rect2i::new(
            -6 + tv.image_size.width / 2,
            -12 + tv.image_size.height / 2,
            12,
            24,
        ),
        Scalar::new(255.0, 255.0, 255.0, 0.),
        2,
        LINE_4,
        0,
    )?;

    for p in point_cloud_ref.point_indices_ref().iter() {
        let xw = p.x(); // world position in m with x facing forward from sensor
        let yw = p.y(); // world position in m with y facing left from sensoe
        let zw = p.z() + 4.75;
        let (x, y) = tv.image_point_top(xw, yw, zw);

        let colour_factor = 255.0 / (tv.world_size.height / 2) as f64;
        let height_factor = 255.0 / 7.75; // maximum height after clip blox

        let xd = xw.abs() as f64;

        let d = (xw * xw + yw * yw).sqrt() as f64;

        imgproc::circle(
            &mut top_view_img,
            Point2i::new(x as i32, y as i32),
            1,
            Scalar::new(
                (height_factor * zw) as f64,
                colour_factor * d,
                colour_factor * (0.5 * tv.world_size.height as f64 - xd),
                0.0,
            ),
            1,
            LINE_4,
            0,
        )?;
    }

    let line_spacing = 10.0;
    let n_markers = tv.world_size.height / line_spacing as i32;
    for i in 0..n_markers {
        let y = ((-(i as f64 * line_spacing) * tv.image_size.height as f64
            / tv.world_size.height as f64)
            + tv.image_size.height as f64) as i32;
        imgproc::line(
            &mut top_view_img,
            Point2i::new(0, y),
            Point2i::new(tv.image_size.width, y),
            Scalar::new(255., 0., 0., 0.),
            1,
            LINE_4,
            0,
        )?;
    }

    if let Some(bbes) = bounding_boxes {
        let color1 = Scalar::new(255.0, 128.0, 128.0, 0.);
        let color2 = Scalar::new(128.0, 128.0, 256.0, 0.);
        for bb in bbes.iter().cloned() {
            // for  (pt1,pt2) in bb.frame_lines() {
            //   let (x1,y1) = tv.image_point_top(pt1.0, pt1.1, pt1.2);
            //   let (x2,y2) = tv.image_point_top(pt2.0, pt2.1, pt2.2);
            //   imgproc::line(&mut top_view_img, Point2i::new(x1 as i32,y1 as i32), Point2i::new(x2 as i32,y2 as i32), Scalar::new(255.,0.,0.,0.),1,LINE_4,0)?;
            // }

            let point_min = bb.point_min();
            let point_max = bb.point_max();

            // if cluster is directly in front of ego use a different color
            let x = point_min.point().x();
            let y = if point_min.point().y() < 0.0 {
                point_max.point().y()
            } else {
                point_min.point().y()
            };
            let color = if x > 0.0 && x < 25.0 && y > -1.5 && y < 1.5 {
                color2
            } else {
                color1
            };

            let (ax, ay) = tv.image_point_top(point_min.point().x(), point_min.point().y(), 0.);
            let (bx, by) = tv.image_point_top(point_max.point().x(), point_max.point().y(), 0.);
            let (dx, dy) = (bx - ax, by - ay);

            imgproc::line(
                &mut top_view_img,
                Point2i::new(ax as i32, ay as i32),
                Point2i::new((ax + dx) as i32, ay as i32),
                color,
                1,
                LINE_4,
                0,
            )?;
            imgproc::line(
                &mut top_view_img,
                Point2i::new(ax as i32, ay as i32),
                Point2i::new(ax as i32, (ay + dy) as i32),
                color,
                1,
                LINE_4,
                0,
            )?;
            imgproc::line(
                &mut top_view_img,
                Point2i::new(bx as i32, by as i32),
                Point2i::new((bx - dx) as i32, by as i32),
                color,
                1,
                LINE_4,
                0,
            )?;
            imgproc::line(
                &mut top_view_img,
                Point2i::new(bx as i32, by as i32),
                Point2i::new(bx as i32, (by - dy) as i32),
                color,
                1,
                LINE_4,
                0,
            )?;

            // display label at the top of the bounding box
            let mut base_line = 0;
            let label = format!("{}", bb.box_id.unwrap_or_default());
            let label_size = get_text_size(&label, FONT_ITALIC, 0.25, 1, &mut base_line)?;
            let top = std::cmp::max(by as i32, label_size.height);
            let y = top - (1.5 * label_size.height as f32).round() as i32;
            let x = bx as i32;
            let rec = Rect2i::new(
                x,
                y,
                (1.5 * label_size.width as f32).round() as i32,
                base_line + (1.5 * label_size.height as f32).round() as i32,
            );

            imgproc::rectangle(&mut top_view_img, rec, color, FILLED, LINE_4, 0)?;
            imgproc::put_text(
                &mut top_view_img,
                &label,
                Point2i::new(bx as i32, by as i32),
                FONT_ITALIC,
                0.25,
                Scalar::default(),
                1,
                LINE_4,
                false,
            )?;
        }
    }

    let window_name = "Top-View Perspective of LiDAR data";
    highgui::named_window(window_name, highgui::WINDOW_AUTOSIZE)?;
    highgui::imshow(window_name, &top_view_img)?;
    if wait {
        highgui::wait_key(0)?;
    }

    Ok(())
}
