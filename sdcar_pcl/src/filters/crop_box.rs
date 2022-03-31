use crate::bounding_box::*;
use crate::point::*;
use crate::point_cloud::*;

use crate::filters::{MaxVec, MinVec};

use rayon::prelude::*;

use std::sync::Arc;

pub fn point_indices_bounding_box<T, U>(
    point_indices_ref: PointIndicesRef<Point<T, U>>,
    min_vec: MinVec,
    max_vec: MaxVec,
    drop: bool, // if true the the bb contained points are excluded otherwise included
) -> PointIndicesRef<Point<T, U>>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    let p_min = T::new(min_vec.x, min_vec.y, min_vec.z);
    let p_max = T::new(max_vec.x, max_vec.y, max_vec.z);

    let point_min = Point::<T, U>::new(p_min, None);
    let point_max = Point::<T, U>::new(p_max, None);

    let bb = BoundingBox::<T, U>::new(None, None, None, point_min, point_max);
    let cropped_point_indices: PointIndices<Point<T, U>> = point_indices_ref
        .par_iter()
        .cloned()
        .filter(|p| {
            let inbb = bb.bounds(p);
            if inbb && !drop {
                true
            } else if !inbb && drop {
                true
            } else {
                false
            }
        })
        .collect();

    Arc::new(cropped_point_indices)
}
