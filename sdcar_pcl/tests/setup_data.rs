use sdcar_pcl::point::*;
use sdcar_pcl::point_cloud::*;


pub fn build_point_indices() -> PointIndices<Point<XYZ,Intensity>> {
  let p1 = PointXYZIntensity::new(-6.2,7.1,0.0,0.0);
  let p2 = PointXYZIntensity::new(-6.3,8.4,0.0,0.0);
  let p3 = PointXYZIntensity::new(-5.2,7.1,0.0,0.0);
  let p4 = PointXYZIntensity::new(-5.7,6.3,0.0,0.0);
  let mut point_indices = PointIndices::<Point<XYZ,Intensity>>::new();
  point_indices.push(PointXYZIntensityRef::new(p1));
  point_indices.push(PointXYZIntensityRef::new(p2));
  point_indices.push(PointXYZIntensityRef::new(p3));
  point_indices.push(PointXYZIntensityRef::new(p4));

  point_indices
}