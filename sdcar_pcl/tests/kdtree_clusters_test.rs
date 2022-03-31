use sdcar_pcl::search::kdtree::*;
use sdcar_pcl::point::*;
use sdcar_pcl::point_cloud::*;
use sdcar_pcl::segmentation::*;
use sdcar_pcl::segmentation::extract_clusters::*;
use std::sync::Arc;

#[test]
fn kdtree_clusters_test() {
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
  let kdtree_ref = Arc::new(kdtree);

  kdtree_ref.clone().print_tree();

  let points =vec!(p1,p2,p3,p4);
  for p1 in points.clone() {
    println!("distances for {:?}", p1.clone());
    for p2 in points.clone() {
      println!("{:?} dist to {:?} = {}", p1.clone().point().array(),p2.clone().point().array(), p1.dist(&p2));
    }
  }

  
  let cloud_ref = Arc::new(PointCloudXYZIntensity::from(point_indices_ref));
  let cluster_tolerance = 0.95;
  let min_cluster_size: usize = 2;
  let max_cluster_size: usize = 4;
  println!("cluster_tolerance: {} min_cluster_size: {} max_cluster_size: {}", cluster_tolerance, min_cluster_size, max_cluster_size);
  let cluster_extract= ClustersExtractXYZIntensity::new(cloud_ref, cluster_tolerance, min_cluster_size, max_cluster_size, kdtree_ref);
  let clusters = cluster_extract.extract();

  println!("found {} clusters", clusters.len());
  for cluster in clusters.iter() {
    println!("cluster length: {} points:{:?}", cluster.len(), cluster);
  }

  assert!(clusters.len() ==1, "should have found 1 cluster not {}", clusters.len());
  assert!(clusters[0].len() == 3, "cluster found should have had 3 points not {}", clusters[0].len());
}