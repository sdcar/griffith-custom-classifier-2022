extern crate opencv;

use opencv::core::{
    min_max_loc, Mat, Point2i, Range, Rect2i, Scalar, Size, Vector, CV_32F,
};
use opencv::dnn;
use opencv::highgui;
use opencv::imgproc::{get_text_size, FILLED, FONT_ITALIC, LINE_4};
use opencv::prelude::*;
use opencv::types::VectorOfMat;
use opencv::Result;
use opencv::imgproc;

use std::fs::File;
use std::io::{BufRead, BufReader, Error};

use crate::bounding_box::*;

pub fn load_coco_classes(path: &str) -> Result<Vec<String>, Error> {
    let mut classes = Vec::<String>::new();
    let input = File::open(path)?;
    let buffered = BufReader::new(input);

    for class in buffered.lines() {
        classes.push(class?);
    }

    Ok(classes)
}

pub struct DNNObjectDetection {
    confidence_threshold: f32,
    nms_threshold: f32,
    net: dnn::Net,
    classes: Vec<String>,
    out_blob_names: Vector<String>,
    ego_point: Point2i, // point used to determine if the bb is on the ego
}

impl DNNObjectDetection {
    pub fn new(
        confidence_threshold: f32,
        nms_threshold: f32,
        classes_file: &str,
        cfg_file: &str,
        darknet_model: &str,
    ) -> Self {
        let classes = load_coco_classes(&classes_file).unwrap();

        // load neural network
        let mut net = dnn::read_net_from_darknet(&cfg_file, &darknet_model).unwrap();
        net.set_preferable_backend(dnn::DNN_BACKEND_CUDA).unwrap();
        net.set_preferable_target(dnn::DNN_TARGET_CUDA_FP16)
            .unwrap();

        // get names of output layers
        let out_layers = net.get_unconnected_out_layers().unwrap();
        let layers_names = net.get_layer_names().unwrap();

        let mut names = Vector::<String>::with_capacity(out_layers.len());
        for i in 0..out_layers.len() {
            let name = layers_names
                .get(out_layers.get(i).unwrap_or_default() as usize - 1)
                .unwrap_or_default();
            names.insert(i, &name).unwrap();
        }

        let ego_point = Point2i::new(630, 660);

        Self {
            confidence_threshold,
            nms_threshold,
            net,
            classes,
            out_blob_names: names,
            ego_point
        }
    }

    pub fn detect_objects(&mut self, img: &Mat, vis: bool, wait: bool, ego_bb: bool) -> Result<ImageBoundingBoxesXYZIntensity> {
        let mut bounding_boxes = ImageBoundingBoxesXYZIntensity::new();
        // generate 4D blob from input image
        let mut net_output = VectorOfMat::new();
        let scalefactor = 1.0 / 255.0;
        let size = Size::new(416, 416);
        let mean = Scalar::new(0.0, 0.0, 0.0, 0.0);
        let swap_rb = false;
        let crop = false;
        let ddepth = CV_32F; // Choose CV_32F or CV_8U
        let blob = dnn::blob_from_image(&img, scalefactor, size, mean, swap_rb, crop, ddepth)?;

        // invoke forward propagation through network
        let mean = Scalar::new(0.0, 0.0, 0.0, 0.0);
        self.net.set_input(&blob, "", 1.0, mean)?;
        self.net.forward(&mut net_output, &self.out_blob_names)?;

        // scan through all teh bounding boxes and keep only the ones with high confidence
        let mut class_ids = Vector::<i32>::new();
        let mut confidences = Vector::<f32>::new();
        let mut bboxes = Vector::<Rect2i>::new();

        for i in 0..net_output.len() {
            // Network produces output blob with a shape NxC where N is a number of
            // detected objects and C is a number of classes + 4 where the first 4
            // numbers are [center_x, center_y, width, height]
            let data = net_output.get(i)?;
            for j in 0..data.rows() {
                let score_col_range = Range::new(5, data.cols())?;
                let scores = data.row(j)?.col_range(&score_col_range)?;
                let mut class_id = Point2i::default();
                let mut confidence: f64 = 0.0;

                // Get the value and location of the maximum score
                min_max_loc(
                    &scores,
                    Some(&mut 0.0),
                    Some(&mut confidence),
                    Some(&mut Point2i::default()),
                    Some(&mut class_id),
                    &Mat::default(),
                )?;
                if confidence as f32 > self.confidence_threshold {
                    let row = data.row(j)?;
                    let center_x = (row.at::<f32>(0)? * img.cols() as f32) as i32;
                    let center_y = (row.at::<f32>(1)? * img.rows() as f32) as i32;
                    let width = (row.at::<f32>(2)? * img.cols() as f32) as i32;
                    let height = (row.at::<f32>(3)? * img.rows() as f32) as i32;

                    let left = center_x - width / 2;
                    let top = center_y - height / 2;

                    class_ids.push(class_id.x);
                    confidences.push(confidence as f32);
                    bboxes.push(Rect2i::new(left, top, width, height));
                }
            }
        }

        // perform non-maxima suppression
        let mut indices = Vector::<i32>::new();
        dnn::nms_boxes(
            &bboxes,
            &confidences,
            self.confidence_threshold,
            self.nms_threshold,
            &mut indices,
            1.0,
            0,
        )?;

        for i in indices {
            let box_id = bounding_boxes.len() as u32;
            let roi = bboxes.get(i as usize)?;
            let class_id = class_ids.get(i as usize)? as u32;
            let class = self.classes[class_id as usize].clone();
            let confidence = confidences.get(i as usize)?;

            let bbox = ImageBoundingBoxXYZIntensity::new(box_id, roi, class_id, class, confidence);
            let mut add_this_bb = true;

            // object detector picks up the bonnet of the ego vehicle
            if ego_bb == false {
              if bbox.contains(self.ego_point) {
                add_this_bb = false;
              }
            }

            if add_this_bb {
            bounding_boxes.push(bbox);
            }
        }

        if vis {
            let mut vis_img = Mat::default();
            img.copy_to(&mut vis_img)?;

            let color = Scalar::new(255.0, 128.0, 128.0, 0.);

            for bb in bounding_boxes.clone() {
                let top = bb.roi.y;
                let left = bb.roi.x;

                imgproc::rectangle(&mut vis_img, bb.roi, color, 1, LINE_4, 0)?;

                let class = self.classes[bb.class_id as usize].clone();
                let label = format!("{} {} {:3.2}", bb.box_id, class, bb.confidence);

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

            let winname = "YOLO V4 Object Detection";
            highgui::named_window(winname, highgui::WINDOW_GUI_NORMAL)?;
            highgui::imshow(winname, &vis_img)?;
            // highgui::wait_key(5)?;
            if wait {
               highgui::wait_key(0)?;
            }
        }

        Ok(bounding_boxes)
    }
}
