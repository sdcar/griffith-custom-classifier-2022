use std::sync::Arc;
use std::sync::Mutex;
use std::sync::RwLock;

use rayon::prelude::*;

use crate::point::*;
use crate::point_cloud::*;

type NodeArc<T,U> =Arc<Node<T,U>>;
type NodeOptional<T,U> = Option<NodeArc<T,U>>;
type NodeRefLock<T,U> = RwLock<NodeOptional<T,U>>;

pub type KdTreeXYZ<T>= KdTree<XYZ,T>;
pub type KdTreePointXYZIntensity = KdTreeXYZ<Intensity>;
pub type KdTreeRef<T,U> = Arc<KdTree<T,U>>;

#[derive(Debug)]
struct Node<T,U> 
where T: PointMeasurable, U:PointDataAccess{
  point: PointRef<T,U>,
  id: u32,
  depth: usize,
  left: NodeRefLock<T,U>,
  right: NodeRefLock<T,U>,
}

impl <T,U> Node<T,U>
where T: PointMeasurable, U:PointDataAccess
{
  fn make_node(node: NodeOptional<T,U>) -> NodeRefLock<T,U> {
    NodeRefLock::new(node)
  }
  pub fn new(point: PointRef<T,U>, id: u32, depth: usize) -> Self{
    Node::<T,U> {point, id, depth, left: Self::make_node(None), right: Self::make_node(None)}
  }
  pub fn point(&self) -> PointRef<T,U> {
    self.point.clone()
  }
}

#[derive(Debug)]
pub struct KdTree<T,U>
where T:PointMeasurable, U:PointDataAccess {
  n_dim: usize,
  root: NodeRefLock<T,U>,
}

impl <T,U> KdTree<T,U>
where T: PointMeasurable, U:PointDataAccess
{
  pub fn new() -> Self{

    // let root = NodeRefCell::<T,U>::new(None);
    let root = Node::<T,U>::make_node(None);

    KdTree::<T,U> {n_dim:<T>::dims(), root}
  }

  fn insert_node (&self, node:&NodeRefLock<T,U>, depth: usize, point:PointRef<T,U>, id:u32) {
    
    {
      let mut node_guard = node.write().unwrap();
      match node_guard.clone() 
      {
        Some(node_ref) => {
          let node_ref_array = node_ref.clone().point().point().array();
          // calculate current dim
          let cd = depth % self.n_dim;
          if point.point().array()[cd] < node_ref_array[cd] {
            self.insert_node(&node_ref.left, depth+1, point.clone(), id);
          } else {
            self.insert_node(&node_ref.right , depth+1, point.clone(), id);
          }
        },
        None =>
        {
          *node_guard = Some(Arc::new(Node::<T,U>::new(point.clone(), id, depth)));
          return;
        }
      }
    } // lock released


  }
    
  pub fn insert(&self, point: PointRef<T,U>, id: u32) {
    self.insert_node(&self.root, 0, point, id);
  } 

  fn search_nodes(&self, point:PointRef<T,U>, node:&NodeRefLock<T,U>, depth: usize, distance_tol:f32, point_indices_ref: &PointIndicesMutexRef<Point<T,U>>)  {
    let node_guard = node.read().unwrap();
    match node_guard.clone() {
      Some(node_ref) => {
        let tp = point.point().array();
        let np = node_ref.point().point().array();

        // TODO make this work with many dimensions other than 3
        if (np[0] >= (tp[0]-distance_tol) && np[0] <= (tp[0]+distance_tol))
        && (np[1] >= (tp[1]-distance_tol) && np[1] <= (tp[1]+distance_tol))
        && (np[2] >= (tp[2]-distance_tol) && np[2] <= (tp[2]+distance_tol))
        {
          let dx = np[0] - tp[0];
          let dy = np[1] - tp[1];
          let dz = np[2] - tp[2];

          let distance = (dx*dx+dy*dy+dz*dz).sqrt();
          if distance < distance_tol {
            point_indices_ref
            .lock()
            .unwrap()
            .push(node_ref.point());
          }
        }

        // check across boundary
        if (tp[depth%self.n_dim]-distance_tol) <= np[depth%self.n_dim] {
          self.search_nodes(point.clone(), &node_ref.left, depth+1, distance_tol, &point_indices_ref)
        }
        if (tp[depth%self.n_dim]+distance_tol) > np[depth%self.n_dim] {
          self.search_nodes(point.clone(), &node_ref.right, depth+1, distance_tol, &point_indices_ref)
        }.clone()
      }
      None => {
        // either an empty tree or reach bottom of leaf node
      }
    };
  }
  // return a vector of point indices in the tree 
  // that are within distance of target
  pub fn search(&self, target: PointRef<T,U>, distance_tol: f32) ->PointIndicesRef<Point<T,U>> {
    let point_indices = Vec::<PointRef<T,U>>::new();
    let point_indices_ref = PointIndicesMutexRef::<Point<T,U>>::new(Mutex::new(point_indices));

    self.search_nodes(target, &self.root, 0, distance_tol, &point_indices_ref);

    let nearest_point_indices = point_indices_ref.lock().unwrap().clone();
    Arc::new(nearest_point_indices)
  }

  pub fn load (&self, point_indices_ref: PointIndicesRef<Point<T,U>>) {
    // let mut id: u32 = 0;
    // for point in point_indices_ref.iter().cloned() {
    //   self.insert(point, id);
    //   id+=1;
    // }
   (0..point_indices_ref.len()).into_par_iter()
   .for_each(|i| {
      self.insert(point_indices_ref[i].clone(), i as u32);
    });
  }

  fn print_tree_node_helper (&self, node:&NodeRefLock<T,U>) {
    let node_guard = node.read().unwrap();
    match node_guard.clone() {
      Some(node_ref) => {
        let node_left = node_ref.left.read().unwrap().clone();
        let node_left_id = match node_left.clone() {
          Some(node_ref) => Some(node_ref.id),
          None => None
        };
        let node_right = node_ref.right.read().unwrap().clone();
        let node_right_id = match node_right.clone() {
          Some(node_ref) => Some(node_ref.id),
          None => None
        };
        println!("id: {} depth: {} {:?} left_id: {:?} right_id {:?}", node_ref.id, node_ref.depth, node_ref.point, node_left_id, node_right_id);
        if node_left.is_some(){
          self.print_tree_node_helper(&node_ref.left);
        }
        if node_right.is_some(){
          self.print_tree_node_helper(&node_ref.right);
        }
      },
      None => {}
    }
  }
  pub fn print_tree(&self) {
    println!("tree start n_dim:{}", self.n_dim);    
    self.print_tree_node_helper(&self.root);
    println!("tree finish");
  }
}


// impl From <PointXYZIntensityIndicesRef> for KdTreePointXYZIntensity {
//   fn from (point_indices_ref: PointXYZIntensityIndicesRef) -> Self {
//     let kdtree = KdTreePointXYZIntensity::new();
//     let mut id: u32 = 0;
//     for point in point_indices_ref.iter().cloned() {
//       kdtree.insert(point, id);
//       id+=1;
//     }

//     kdtree
//   }
// }

impl From <PointXYZIntensityIndicesRef> for KdTreePointXYZIntensity {
  fn from (point_indices_ref: PointXYZIntensityIndicesRef) -> Self {
    let kdtree = KdTreePointXYZIntensity::new();
    kdtree.load(point_indices_ref);
    kdtree
  }
}

