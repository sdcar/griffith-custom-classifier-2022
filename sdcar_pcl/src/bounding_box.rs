use crate::filters::get_min_max;
use crate::point::*;
use crate::point_cloud::*;

use std::sync::Arc;

pub type BoundingBoxRef<T, U> = Arc<BoundingBox<T, U>>;
pub type BoundingBoxXYZIntensity = BoundingBox<XYZ, Intensity>;
pub type BoundingBoxXYZIntensityRef = Arc<BoundingBoxXYZIntensity>;

pub type BoundingBoxes<T, U> = Vec<BoundingBoxRef<T, U>>;
pub type BoundingBoxesRef<T, U> = Arc<BoundingBoxes<T, U>>;
pub type BoundingBoxesXYZIntensityRef = BoundingBoxesRef<XYZ, Intensity>;

#[derive(Debug, Clone, Copy, Hash)]
pub struct BBLine<T>
where
    T: PointMeasurable,
{
    a: T,
    b: T,
}

pub type PointScreenCoord = (f32, f32, f32);
pub type LineScreenCoord = (PointScreenCoord, PointScreenCoord);

pub trait HasDrawableLine<T> {
    fn new(a: &T, b: &T) -> Self;
    // returns the co-ordinates in screen space as integers
    fn line_screen(&self) -> LineScreenCoord;
    fn bounding_box_lines_screen(&self) -> Vec<LineScreenCoord>;
}

impl<T> HasDrawableLine<T> for BBLine<T>
where
    T: PointMeasurable,
{
    fn new(a: &T, b: &T) -> Self {
        let at = a.tuple();
        let bt = b.tuple();
        Self {
            a: T::new(at.0, at.1, at.2),
            b: T::new(bt.0, bt.1, bt.2),
        }
    }
    fn line_screen(&self) -> LineScreenCoord {
        let a = self.a.tuple();
        let b = self.b.tuple();
        let a_out = (a.1, a.2, a.0);
        let b_out = (b.1, b.2, b.0);
        (a_out, b_out)
    }
    fn bounding_box_lines_screen(&self) -> Vec<LineScreenCoord> {
        let mut lines = Vec::<LineScreenCoord>::with_capacity(12);
        let (a, b) = self.line_screen();
        let (ax, ay, az) = a;
        let (bx, by, bz) = b;
        let (dx, dy, dz) = (bx - ax, by - ay, bz - az);

        lines.push(((ax, ay, az), (ax + dx, ay, az)));
        lines.push(((ax, ay, az), (ax, ay + dy, az)));
        lines.push(((ax, ay, az), (ax, ay, az + dz)));
        lines.push(((ax + dx, ay + dy, az), (ax, ay + dy, az)));
        lines.push(((ax + dx, ay + dy, az), (ax + dx, ay, az)));
        lines.push(((ax + dx, ay + dy, az), (ax + dx, ay + dy, az + dz)));
        lines.push(((bx, by, bz), (bx - dx, by, bz)));
        lines.push(((bx, by, bz), (bx, by - dy, bz)));
        lines.push(((bx, by - dy, bz), (bx - dx, by - dy, bz)));
        lines.push(((bx, by - dy, bz), (bx, by - dy, bz - dz)));
        lines.push(((bx - dx, by, bz), (bx - dx, by - dy, bz)));
        lines.push(((bx - dx, by, bz), (bx - dx, by, bz - dz)));

        lines
    }
}
#[derive(Debug, Clone, Copy, Hash)]
pub struct BoundingBox<T, U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    pub box_id: Option<u32>,
    pub cluster_id: Option<u32>,
    pub track_id: Option<u32>,
    point_min: Point<T, U>,
    point_max: Point<T, U>,
}

pub trait HasBoundingBox<T, U> {
    fn new(
        box_id: Option<u32>,
        cluster_id: Option<u32>,
        track_id: Option<u32>,
        point_min: Point<T, U>,
        point_max: Point<T, U>,
    ) -> Self;
    fn dist(&self) -> f32;
    fn bounds(&self, point_ref: &Arc<Point<T, U>>) -> bool;
    fn frame_lines(&self) -> Vec<LineScreenCoord>;
    fn point_min(&self) -> Point<T, U>;
    fn point_max(&self) -> Point<T, U>;
    fn dxyz(&self) -> (f32, f32, f32);
}

impl<T, U> HasBoundingBox<T, U> for BoundingBox<T, U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    fn new(
        box_id: Option<u32>,
        cluster_id: Option<u32>,
        track_id: Option<u32>,
        point_min: Point<T, U>,
        point_max: Point<T, U>,
    ) -> Self {
        Self {
            box_id,
            cluster_id,
            track_id,
            point_min,
            point_max,
        }
    }
    fn dist(&self) -> f32 {
        self.point_min.point().dist(&self.point_max.point())
    }

    fn bounds(&self, point_ref: &Arc<Point<T, U>>) -> bool {
        // TODO modify this to work with other dimensions than XYZ
        let a = self.point_min.point().array();
        let b = self.point_max.point().array();

        let point = point_ref.point().array();
        if (a[0] <= point[0] && point[0] <= b[0])
            && (a[1] <= point[1] && point[1] <= b[1])
            && (a[2] <= point[2] && point[2] <= b[2])
        {
            true
        } else {
            false
        }
    }
    fn frame_lines(&self) -> Vec<LineScreenCoord> {
        let bbline = BBLine::<T>::new(self.point_min.point(), self.point_max.point());
        bbline.bounding_box_lines_screen()
    }
    fn point_min(&self) -> Point<T, U> {
        self.point_min.clone()
    }
    fn point_max(&self) -> Point<T, U> {
        self.point_max.clone()
    }
    fn dxyz(&self) -> (f32, f32, f32) {
        let a = self.point_max.point().tuple();
        let b = self.point_min.point().tuple();
        (a.0 - b.0, a.1 - b.1, a.2 - b.2)
    }
}

impl<T, U> From<PointIndicesRef<Point<T, U>>> for BoundingBox<T, U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    fn from(point_indices_ref: PointIndicesRef<Point<T, U>>) -> Self {
        let cloud = PointCloud::<Point<T, U>>::from(point_indices_ref);

        let (min_vec, max_vec) = get_min_max(&cloud);
        let point_min = Point::<T, U>::new(T::new(min_vec.x, min_vec.y, min_vec.z), None);
        let point_max = Point::<T, U>::new(T::new(max_vec.x, max_vec.y, max_vec.z), None);

        BoundingBox::<T, U>::new(None, None, None, point_min, point_max)
    }
}

impl<T, U> From<PointCloudRef<Point<T, U>>> for BoundingBox<T, U>
where
    T: PointMeasurable,
    U: PointDataAccess,
{
    fn from(cloud: PointCloudRef<Point<T, U>>) -> Self {
        let (min_vec, max_vec) = get_min_max(&cloud);
        let point_min = Point::<T, U>::new(T::new(min_vec.x, min_vec.y, min_vec.z), None);
        let point_max = Point::<T, U>::new(T::new(max_vec.x, max_vec.y, max_vec.z), None);

        BoundingBox::<T, U>::new(None, None, None, point_min, point_max)
    }
}
