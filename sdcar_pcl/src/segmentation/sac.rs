extern crate nalgebra;

use crate::point::*;
use crate::point_cloud::*;

use nalgebra::Vector3;
use rand::distributions::{Distribution, Uniform};
use rayon::prelude::*;
use std::sync::Arc;

pub fn ransac(
    in_cloud: &PointCloudXYZIntensity,
    max_iterations: usize,
    distance_to_l: f32,
) -> PointXYZIntensityIndicesRef {
    let point_indices_ref = in_cloud.point_indices_ref();

    let indices_uniform = Uniform::from(0..point_indices_ref.len());

    let random_point_sample = || -> PointXYZIntensityRef {
        let mut rng = rand::thread_rng();
        let indices = indices_uniform.sample(&mut rng);
        point_indices_ref[indices].clone()
    };

    let sample_and_fit_line = || -> PointXYZIntensityIndicesRef {
        // create 3 random inlier samples
        let mut inlier_samples = PointXYZIntensityIndices::new();
        inlier_samples.push(random_point_sample());
        inlier_samples.push(random_point_sample());
        inlier_samples.push(random_point_sample());

        // measure distance between every point and fitted line
        let (x1, y1, z1) = inlier_samples[0].point().tuple();
        let (x2, y2, z2) = inlier_samples[1].point().tuple();
        let (x3, y3, z3) = inlier_samples[2].point().tuple();

        let v1 = Vector3::<f32>::new(x2 - x1, y2 - y1, z2 - z1);
        let v2 = Vector3::<f32>::new(x3 - x1, y3 - y1, z3 - z1);

        let nv = v1.cross(&v2);
        let (i, j, k) = (nv[0], nv[1], nv[2]);

        let (a, b, c) = (i, j, k);
        let d = -(i * x1 + j * y1 + k * z1);
        let e = (a * a + b * b + c * c).sqrt();

        let inlier_filter = point_indices_ref.par_iter().cloned().filter(|p| {
            // include the inlier sample
            if inlier_samples.contains(p) {
                return true;
            };

            let (x, y, z) = p.point().tuple();
            let distance = (a * x + b * y + c * z + d).abs() / e;
            // if distance is smaller than threshold, count it as inlier
            distance <= distance_to_l
        });
        let inliers: PointIndices<PointXYZIntensity> = inlier_filter.collect();
        Arc::new(inliers)
    };

    let inlier_point_indices_refs_iter = (0..max_iterations)
        .into_par_iter()
        .map(|_x| sample_and_fit_line());

    let inlier_point_indices_refs: Vec<PointXYZIntensityIndicesRef> =
        inlier_point_indices_refs_iter.collect();

    match inlier_point_indices_refs
        .par_iter()
        .cloned()
        .max_by_key(|x| x.len())
    {
        Some(a) => a,

        None => PointXYZIntensityIndicesRef::new(PointXYZIntensityIndices::new()),
    }
}
