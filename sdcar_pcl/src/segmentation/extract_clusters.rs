use crate::search::kdtree::*;
use crate::point_cloud::*;
use crate::point::*;

use rayon::prelude::*;

use std::sync::Arc;
use std::sync::RwLock;
use dashmap::{DashSet,DashMap};
use rand::distributions::{Distribution,Uniform};

pub type ClusterRef<T,U> = Arc<RwLock<Cluster<T,U>>>;
pub type Clusters<T,U>  = Vec<PointIndicesRef<Point<T,U>>>;
pub type ClustersRef<T,U>  = Arc<Clusters<T,U>>;
// pub type ClusterPointIndicesRef<T,U>  = Arc<Vec<PointIndicesRef<Point<T,U>>>>;


// type PointProcessed<T,U> = Arc<Mutex<PointHashSet<T,U>>>;
type PointProcessed<T,U> = DashSet<PointRef<T,U>>;
type PointProcessedRef<T,U> = Arc<PointProcessed<T,U>>;

type PointLocked<T,U> = DashMap<PointRef<T,U>,usize>;
type PointLockedRef<T,U> = Arc<PointLocked<T,U>>;

pub type ClustersExtractXYZIntensity = EuclideanClustersExtract<XYZ,Intensity>;

#[derive(Debug,Clone)]
pub struct Cluster<T,U>
where T: PointMeasurable, U: PointDataAccess {
  point_indices_ref: Arc<RwLock<PointIndices<Point<T,U>>>>,
  point_processed_ref: PointProcessedRef<T,U>
}

impl<T,U> Cluster<T,U>
where T: PointMeasurable, U: PointDataAccess {
  fn new(point_processed_ref:PointProcessedRef<T,U>) -> Self{
    let point_indices:PointIndices<Point<T,U>> = PointIndices::<Point<T,U>>::new();
    let point_indices_ref = Arc::new(RwLock::new(point_indices));
    Self {point_indices_ref, point_processed_ref}
  }
  fn push(&mut self, point_ref: PointRef<T,U>)  {
    let mut point_indices_guard = self.point_indices_ref.write().unwrap();
    point_indices_guard.push(point_ref.clone());
  }
  fn is_point_processed(&self, point_ref: PointRef<T,U>) -> bool {
    self.point_processed_ref.contains(&point_ref)
  }
  fn point_indices_ref(&self) -> PointIndicesRef<Point<T,U>> {
    // let pi:PointIndices<Point<T,U>> = self.point_indices.drain(..).collect();
    Arc::new(self.point_indices_ref.read().unwrap().clone())
  }
  fn num_points(&self) -> usize {
    self.point_indices_ref.read().unwrap().len()
  }
}

#[derive(Debug)]
pub struct EuclideanClustersExtract<T,U>
where T: PointMeasurable, U: PointDataAccess {
  cloud_ref: PointCloudRef<Point<T,U>>,
  cluster_tolerance: f32,
  min_cluster_size: usize,
  max_cluster_size: usize,
  kdtree_ref: KdTreeRef<T,U>,
}

impl<T,U> EuclideanClustersExtract<T, U>
where T: PointMeasurable, U: PointDataAccess
{
    pub fn new(cloud_ref: PointCloudRef<Point<T,U>>, cluster_tolerance: f32, min_cluster_size: usize, max_cluster_size: usize, kdtree_ref: KdTreeRef<T,U>) -> Self {
      Self {cloud_ref, cluster_tolerance, min_cluster_size, max_cluster_size, kdtree_ref }
    }

    fn cluster_builder(&self, point_ref: PointRef<T,U>, cluster_ref: &ClusterRef<T,U>) {
      {
        let mut cluster_guard = cluster_ref.write().unwrap();
        if cluster_guard.is_point_processed(point_ref.clone()) {
          return;
        } else {
          cluster_guard.point_processed_ref.insert(point_ref.clone());
          cluster_guard.push(point_ref.clone());
          if cluster_guard.num_points() >= self.max_cluster_size  {
            return;
          }
        }
      }// lock released

      let nearest_points_ref = self.kdtree_ref.search(point_ref.clone(), self.cluster_tolerance);

      // not interested in loan points not near any other
      if nearest_points_ref.len() == 1 {
        return;
      }

      (0..nearest_points_ref.len())
      .into_par_iter()
      .for_each(|i| {
        let p = nearest_points_ref.get(i).unwrap();
        // println!("{:?} nearest point {:?}", point_ref.clone().point().array(), p.clone().point().array());
        self.cluster_builder( p.clone(), &cluster_ref);
      });
    }

    fn cluster_search (&self, point_ref: PointRef<T,U>, point_processed_ref:PointProcessedRef<T,U>, point_locked_ref: PointLockedRef<T,U>, thread_id:usize) -> PointIndicesRef<Point<T,U>>{
      let cluster_ref: ClusterRef<T,U> = Arc::new(RwLock::new(Cluster::<T,U>::new(point_processed_ref.clone())));

      if point_locked_ref.get(&point_ref.clone()).unwrap().clone() == thread_id {
        self.cluster_builder(point_ref, &cluster_ref);
      }

      let cluster_guard = cluster_ref.read().unwrap();

      // println!("cluster built with {} points", cluster_guard.num_points());
//
      // for point in point_processed_ref.iter() {
      //   println!("point_processed {:?}", point.clone());
      // }
      let point_indices_ref = cluster_guard.point_indices_ref();
      point_indices_ref
    }


    pub fn extract_parallel(&self) -> Clusters<T,U> {
      // let mut clusters = Clusters::<T,U>::new();
      let num_points = self.cloud_ref.point_indices_ref().len();

      let point_processed_ref:PointProcessedRef<T,U> =  Arc::new(PointProcessed::<T,U>::with_capacity(num_points));
      let point_locked_ref: PointLockedRef<T,U> = Arc::new(PointLocked::<T,U>::new());


      let point_indices_range = 0..self.cloud_ref.point_indices_ref().len();

      let clusters:Clusters<T,U> = point_indices_range.clone()
        .into_par_iter()
        .map_init(|| {
          let rng = rand::thread_rng();
          let point_indicees_uniform = Uniform::from(point_indices_range.clone());
          (rng, point_indicees_uniform)
         },|(rng, point_indicees_uniform), i| -> PointIndicesRef<Point<T,U>>{
          let n = point_indicees_uniform.sample(rng);
          let point_ref = self.cloud_ref.point(n);

          // slight chance as running in parallel, that the point has been processed
          // so dont search just return a point indices that will be filtered out
          match point_processed_ref.contains(&point_ref) || point_locked_ref.contains_key(&point_ref) {
            true => Arc::new(PointIndices::<Point<T,U>>::new()),
            false => {
              point_locked_ref.insert(point_ref.clone(), i);
              let pir = self.cluster_search(point_ref.clone(), point_processed_ref.clone(),point_locked_ref.clone(), i);
              point_locked_ref.remove(&point_ref);
              pir
            }

          }
        })
        .filter(|x| {
          x.len() >= self.min_cluster_size
        })
        .collect();

      clusters
    }

    pub fn extract(&self) -> Clusters<T,U>{

      let num_points = self.cloud_ref.point_indices_ref().len();

      let point_processed_ref:PointProcessedRef<T,U> =  Arc::new(PointProcessed::<T,U>::with_capacity(num_points));
      let point_locked_ref: PointLockedRef<T,U> = Arc::new(PointLocked::<T,U>::new());


      let point_indices_range = 0..self.cloud_ref.point_indices_ref().len();

      let clusters:Clusters<T,U> = point_indices_range.clone()
        .into_iter()
        .map(|i| -> PointIndicesRef<Point<T,U>>{
          let point_ref = self.cloud_ref.point(i);

          // slight chance as running in parallel, that the point has been processed
          // so dont search just return a point indices that will be filtered out
          match point_processed_ref.contains(&point_ref) || point_locked_ref.contains_key(&point_ref) {
            true => Arc::new(PointIndices::<Point<T,U>>::new()),
            false => {
              point_locked_ref.insert(point_ref.clone(), i);
              let pir = self.cluster_search(point_ref.clone(), point_processed_ref.clone(),point_locked_ref.clone(), i);
              point_locked_ref.remove(&point_ref);
              pir
            }

          }
        })
        .filter(|x| {
          x.len() >= self.min_cluster_size
        })
        .collect();

      clusters
    }

}


