use sdcar_pcl::search::kdtree::*;
use sdcar_pcl::point::*;
use sdcar_pcl::point_cloud::*;
use std::sync::Arc;

#[test]
fn kdtree_test() {

  let p1 = PointXYZIntensity::new(-6.2,7.1,0.0,0.0);
  let p2 = PointXYZIntensity::new(-6.3,8.4,0.0,0.0);
  let p3 = PointXYZIntensity::new(-5.2,7.1,0.0,0.0);
  let p4 = PointXYZIntensity::new(-5.7,6.3,0.0,0.0);
  let mut point_indices = PointIndices::<PointXYZIntensity>::new();
  point_indices.push(PointXYZIntensityRef::new(p1));
  point_indices.push(PointXYZIntensityRef::new(p2));
  point_indices.push(PointXYZIntensityRef::new(p3));
  point_indices.push(PointXYZIntensityRef::new(p4));

  let point_indices_ref: PointXYZIntensityIndicesRef = Arc::new(point_indices);

  let num_points = point_indices_ref.len();
  assert!(num_points==4, "{} points loaded", num_points);

  let kdtree = KdTree::from(point_indices_ref.clone());

  kdtree.print_tree();

  // let tp = PointXYZIntensity::new(-6.0,7.0,0.0,0.0);
  let tp = p2.clone();
  println!("distances");
  for p in point_indices_ref.iter() {
    println!("{:?} target {:?} distance {}", p,tp, p.dist(&tp));
  }
  let target = Arc::new(tp);
  // let distance_tol =0.78;
  let distance_tol =1.75;
  let nearby = kdtree.search(target.clone(), distance_tol);
  println!("search");
  for p in nearby.iter() {
    println!("{:?} nearby point {:?}", target.clone(), p);
  }

  assert!(nearby.len()==3, "target point nearby count should be 3 not {}",nearby.len() )

}