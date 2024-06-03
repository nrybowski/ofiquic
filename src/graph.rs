use std::collections::{HashMap, HashSet, BinaryHeap};
use core::hash::Hash;
use std::cmp::Ord;

pub type WeightedNeighbors<T, U> = Vec<(T, U)>;
pub type WeightedHashSuccessors<T, U> = HashMap<T, WeightedNeighbors<T, U>>;

pub type Neighbors<T> = Vec<T>;
pub type HashSuccessors<T> = HashMap<T, Neighbors<T>>;

pub trait Successors<T, U> {
    fn successors(&self, from: &T) -> WeightedNeighbors<&T, &U>;
}

pub trait DAGOrder<T> {
    fn depths(&self, root: &T) -> (HashMap<T, usize>, HashMap<usize, Vec<T>>);
    fn order(&self, root: &T, order_type: u8) -> Vec<T>;
}

/// Test doc
///
trait ReverseParenthood<T> {
    fn reverse(&self) -> Self;
}

impl<T: Ord + Hash + Copy> ReverseParenthood<T> for HashSuccessors<T> {

    fn reverse(&self) -> Self {
        let mut hash: Self = Self::new();
        
        self
        .iter()
        .for_each(|(node, parents)| 
            parents
            .iter()
            .for_each(|parent|
                if parent != node {
                    hash.entry(*parent).or_insert_with(Vec::new).push(*node)
                }
            )
        );

        hash
    }

}

impl<T: Ord + Hash + Copy + std::fmt::Debug> DAGOrder<T> for HashSuccessors<T> {


    /// Returns a tuple with two entries
    /// First, a hashmap whose keys are node IDs and the value, the depth of the node in the rspDAG
    /// Second, a hashmap whose keys are depths and the value, a list of nodes at that depth
    fn depths(&self, root: &T) -> (HashMap<T, usize>, HashMap<usize, Vec<T>>) {

        fn dfs<T: Ord + Hash + Copy>(successors: &HashSuccessors<T>, depth: usize, current: &T, visited: &mut HashMap<T, usize>) {
            let old_depth = visited.entry(*current).or_insert(depth);
            if *old_depth < depth {
                *old_depth = depth;
            }
            if let Some(children) = successors.get(current) {
                children.iter().for_each(|child| dfs(successors, depth + 1, child, visited));
            }
        }

        let reversed = ReverseParenthood::reverse(self);
        let mut visited: HashMap<T, usize> = HashMap::new();
        dfs(&reversed, 0, root, &mut visited);

        // Define mapping depth: node ids
        let mut by_depth: HashMap<usize, Vec<T>> = HashMap::new();
        visited
        .iter()
        .for_each(|(node, depth)| by_depth.entry(*depth).or_insert_with(Vec::new).push(*node));

        // tie-break if multiple nodes at the same depth, sort them by increasing node id
        by_depth.iter_mut().for_each(|(_, nodes)| nodes.sort());
        
        (visited, by_depth)
    }


    fn order(&self, root: &T, order_type: u8) -> Vec<T> {
        let mut by_depth: HashMap<usize, Vec<T>> = HashMap::new();

        let (pre_depths, pre_by_depths) = self.depths(root);
        
        let mut order: Vec<&usize> = pre_by_depths.keys().collect();
        order.sort();

        // Flatten node ids
        order
        .iter()
        .rev()
        .flat_map(|depth| &pre_by_depths[&depth] )
        .collect::<Vec<&T>>()
        .iter()
        .map(|&entry| *entry)
        .collect::<Vec<T>>()
    }
}

pub trait DAG<T, U> {
    fn spdag(&self, root: &T) -> Option<HashSuccessors<T>>;
    fn revert(&self) -> Self;
    fn rspdag(&self, root: &T) -> Option<HashSuccessors<T>>;
    fn order_failure(&self, root: &T, neighbor: &T, metric: u16, order_type: u8) -> Vec<T>;
}

impl<T: Ord + Hash + Copy, U: Copy> Successors<T, U> for WeightedHashSuccessors<T, U> {
    
    fn successors(&self, from: &T) -> WeightedNeighbors<&T, &U> {
        self.get(from)
            .unwrap()
            .iter()
            .map(|(node, cost)| (node, cost))
            .collect()
    }
}

impl<T: Copy + Ord + Hash + std::fmt::Debug> DAG<T, u16> for WeightedHashSuccessors<T, u16> {

    fn spdag(&self, root: &T) -> Option<HashSuccessors<T>> {
        dijkstra(self, root)
    }

    fn revert(&self) -> Self {
        let mut ret: Self = Self::with_capacity(self.len());
        for (src, successors) in self.iter() {
            let mut line: WeightedNeighbors<T, u16> = WeightedNeighbors::new();
            for (dst, _) in successors {
                self[dst].iter().filter_map(|(child, cost)| if &child == &src {Some((*dst, *cost))} else {None}).for_each(|x| line.push(x));
            }
            ret.insert(*src, line);
        }
        ret
    }

    fn rspdag(&self, root: &T) -> Option<HashSuccessors<T>> {
        self.revert().spdag(root)
    }

    fn order_failure(&self, root: &T, neighbor: &T, metric: u16, order_type: u8) -> Vec<T> {
        let pre_successors = self;
        let pre_rspdag = self.rspdag(root).unwrap();
        
        let order = if order_type == 0 {
            let mut order = DAGOrder::order(&pre_rspdag, neighbor, order_type);
            order.pop();
            order
        } else {
            let post_successors: WeightedHashSuccessors<T, u16> = 
                pre_successors.iter()
                .map(|(&head_rid, &ref succ)| (
                    head_rid.clone(),
                    succ.iter()
                        .map(|(tail_rid, _metric)| (
                            tail_rid.clone(),
                            if head_rid == *neighbor && tail_rid == root {
                                // TODO: get metric from OSPF LSA
                                metric
                            } else {
                                _metric.clone()
                            })
                        ).collect()
                    )
                ).collect();

            //println!("pre {:#?}\npost{:#?}", pre_successors, post_successors);

            let pre_rspdag = DAG::rspdag(pre_successors, root).unwrap();
            let (pre_depths, pre_depths_by_order) = DAGOrder::depths(&pre_rspdag, neighbor);

            /* Order oFIB candidates by decreasing depth */
            let mut pre_ordered_depths = pre_depths_by_order.keys().collect::<Vec<&usize>>();
            pre_ordered_depths.sort();
            pre_ordered_depths.reverse();
        
            let post_rspdag = DAG::rspdag(&post_successors, root).unwrap();
            let (post_depths, post_depths_by_order) = DAGOrder::depths(&post_rspdag, root);

            //println!("pre {:#?}\npost {:#?}", pre_rspdag, post_rspdag);

            let mut order: Vec<T> = Vec::new();

            for depth in pre_ordered_depths {
                for candidate in &pre_depths_by_order[depth] {
                    //println!("candidate {:#?}", candidate);
                    for peer in &pre_rspdag[&candidate] {
                        //println!(" peer {:#?}", peer);
                        for peer_succ in &post_rspdag[&peer] {
                            //println!("  peer_succ {:#?}", peer_succ);
                            if peer_succ == candidate {
                                //println!("   match");
                                if !order.contains(&candidate) { order.push(candidate.clone()); }
                                if peer != neighbor && !order.contains(&peer) { order.push(peer.clone()); }
                            }
                        }
                    }
                }
            }

//            for depth in pre_ordered_depths {
//                let candidates = &pre_depths_by_order[depth];
//                let pre_depth = depth + 1;
//                for candidate in candidates {
//                    let post_depth = post_depths[candidate];
//                    if post_depth > pre_depth {
//                        // If a given node goes deeper in the post-convergent RSPDAG than in the pre-convergence RSPDAG
//                        //println!("{:#?} {:#?}", candidate, post_rspdag[candidate]);
//                        for c in &post_rspdag[candidate] {
//                            if pre_depths.contains_key(c) && !order.contains(c) {order.push(*c);}
//                        }
//                        // //println!("{:#?}: pre_depth {} post_depth {} : {:#?}", candidate, pre_depth, post_depth, x);
//                        // order.extend(x);
//                        order.push(*candidate);
//                    
//                    }
//                }
//            }
//
//            order = order.iter().filter_map(|_rid| if _rid != neighbor {Some(*_rid)} else {None}).collect();
            order
        };
        order
    }
}

pub fn dijkstra<'a, T: Ord + Hash + Copy>(graph: &WeightedHashSuccessors<T, u16>, start: &'a T) -> Option<HashSuccessors<T>> {
    let mut heap: BinaryHeap<(i32, (&T, &T))> = BinaryHeap::new();
    let mut visited: HashSet<&T> = HashSet::new();
    let mut cost_to_reach: HashMap<&T, i32> = HashMap::new();
    let mut predecessors: HashMap<&T, Vec<&T>> = HashMap::new();

    heap.push((0, (start, start)));
    while !heap.is_empty() {
        let (cost, (current, from)) = match heap.pop() {
            Some(infos) => infos,
            None => return None,
        };

        if visited.contains(current) {
            // Maybe ECMP?
            match cost_to_reach.get(current) {
                None => continue,
                Some(optimal_cost) => {
                    if *optimal_cost == cost {
                        // This is ECMP!
                        predecessors.entry(current).or_insert_with(Vec::new).push(from);
                    }
                }
            }
            // Do not need to expand the node, we already did it
            continue;
        }

        visited.insert(current);
        predecessors.entry(current).or_insert_with(Vec::new).push(from);
        cost_to_reach.insert(current, cost);

        // Add all neighbours
        for (neigh, local_cost) in graph
            .successors(current)
            .iter()
            .filter(|(neigh, _)| !visited.contains(neigh))
        {
            heap.push((cost - **local_cost as i32, (neigh, current)));
        }
    }
    Some(predecessors.iter().map(|(key, value)| (*key.clone(), value.iter().map(|&v| *v).collect::<Vec<T>>())).collect())
}

#[cfg(test)]
mod triangle {
    use crate::{WeightedHashSuccessors, DAG, HashSuccessors, graph::DAGOrder};
    use std::collections::HashMap;

    fn gen_symetric() -> WeightedHashSuccessors<usize, u16> {
        let mut topo = WeightedHashSuccessors::new();
        topo.insert(1, vec![(2, 1), (3, 1)]);
        topo.insert(2, vec![(1, 1), (3, 1)]);
        topo.insert(3, vec![(1, 1), (2, 1)]);
        topo
    }

    fn gen_symetric_order() -> HashMap<usize, Vec<usize>> {
        let mut ret = HashMap::new();
        ret.insert(1, vec![2, 3, 1]);
        ret.insert(2, vec![1, 3, 2]);
        ret.insert(3, vec![1, 2, 3]);
        ret
    }

    #[test]
    fn test_order_symetric() {
        let topo = gen_symetric();
        let expected = gen_symetric_order();
        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let result = DAGOrder::order(&rspdag, root, 0);
            assert_eq!(result, expected[root]);
        }
    }
}

/// Module's tests on the house topology
///
/// Each test comes in two flavours, one for the classical house topology (including ECMP paths)
/// and one for the house with an asymmetric link between nodes 1 and 3, i.e.:
/// 1 - (2) -> 3
/// 3 - (6) -> 1
/// This second topology allow cases where Node 1 can reach 3 via 3 and 2, but 3 can reach Node 1
/// only via Node 2.
///
/// TODO: Tests for parallel links between nodes.
///
#[cfg(test)]
mod house {
    use crate::{WeightedHashSuccessors, DAG, HashSuccessors, graph::DAGOrder};
    use std::collections::HashMap;

    fn gen_house() -> WeightedHashSuccessors<usize, u16> {
        let mut topo: WeightedHashSuccessors<usize, u16> = WeightedHashSuccessors::new();
        topo.insert(1, vec![(2, 1), (3, 10)]);
        topo.insert(2, vec![(1, 1), (3, 1), (5, 10), (4, 1)]);
        topo.insert(3, vec![(1, 10), (2, 1), (5, 1), (6, 1)]);
        topo.insert(4, vec![(2, 1), (6, 1)]);
        topo.insert(5, vec![(2, 10), (3, 1)]);
        topo.insert(6, vec![(4, 1), (3, 1)]);
        topo
    }

    fn gen_asymetric_house() -> WeightedHashSuccessors<usize, u16> {
        // 1 - (2) -> 3
        // 3 - (6) -> 1
        let mut topo = gen_house();
        topo.get_mut(&1).unwrap()[1].1 = 2;
        topo.get_mut(&3).unwrap()[0].1 = 6;
        topo
    }

    fn gen_asymetric_house_spts() -> Vec<HashSuccessors<usize>> {
        let mut spts = gen_house_spts();
        let node1 = spts.get_mut(0).unwrap();
        let node1_3 = node1.get_mut(&3).unwrap();
        node1_3.push(1);
        node1_3.sort();
        spts
    }

    fn gen_house_spts() -> Vec<HashSuccessors<usize>> {
        let mut spts: Vec<HashSuccessors<usize>> = Vec::with_capacity(6);
        spts.push(HashMap::from([
            (1, vec![1]),
            (2, vec![1]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![2]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3]),
            (3, vec![3]),
            (4, vec![2, 6]),
            (5, vec![3]),
            (6, vec![3])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![4]),
            (3, vec![2, 6]),
            (4, vec![4]),
            (5, vec![3]),
            (6, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3]),
            (3, vec![5]),
            (4, vec![2, 6]),
            (5, vec![5]),
            (6, vec![3])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3, 4]),
            (3, vec![6]),
            (4, vec![6]),
            (5, vec![3]),
            (6, vec![6])
        ]));
        spts
    }

    fn gen_symetric_spdags_max_depths() -> HashMap<usize, HashMap<usize,usize>>{
        let mut expected: HashMap<usize, HashMap<usize,usize>> = HashMap::new();
        expected.insert(1, HashMap::from([(1,0), (2,1), (3,2), (4,2), (5,3), (6,3)]));
        expected.insert(2, HashMap::from([(1,1), (2,0), (3,1), (4,1), (5,2), (6,2)]));
        expected.insert(3, HashMap::from([(1,2), (2,1), (3,0), (4,2), (5,1), (6,1)]));
        expected.insert(4, HashMap::from([(1,2), (2,1), (3,2), (4,0), (5,3), (6,1)]));
        expected.insert(5, HashMap::from([(1,3), (2,2), (3,1), (4,3), (5,0), (6,2)]));
        expected.insert(6, HashMap::from([(1,3), (2,2), (3,1), (4,1), (5,2), (6,0)]));
        expected
    }

    fn gen_symetric_order() -> HashMap<usize, Vec<usize>> {
        let mut ret = HashMap::new();
        ret.insert(1, vec![5, 6, 3, 4, 2, 1]);
        ret.insert(2, vec![5, 6, 1, 3, 4, 2]);
        ret.insert(3, vec![1, 4, 2, 5, 6, 3]);
        ret.insert(4, vec![5, 1, 3, 2, 6, 4]);
        ret.insert(5, vec![1, 4, 2, 6, 3, 5]);
        ret.insert(6, vec![1, 2, 5, 3, 4, 6]);
        ret
    }

    fn _gen_asymetric_spdags_max_depths() -> HashMap<usize, HashMap<usize,usize>>{
        let mut expected: HashMap<usize, HashMap<usize,usize>> = HashMap::new();
        expected.insert(1, HashMap::from([(1,0), (2,1), (3,2), (4,2), (5,3), (6,3)]));
        expected.insert(2, HashMap::from([(1,1), (2,0), (3,1), (4,1), (5,2), (6,2)]));
        expected.insert(3, HashMap::from([(1,2), (2,1), (3,0), (4,2), (5,1), (6,1)]));
        expected.insert(4, HashMap::from([(1,2), (2,1), (3,2), (4,0), (5,3), (6,1)]));
        expected.insert(5, HashMap::from([(1,3), (2,2), (3,1), (4,3), (5,0), (6,2)]));
        expected.insert(6, HashMap::from([(1,3), (2,2), (3,1), (4,1), (5,2), (6,0)]));
        expected
    }

    #[test]
    fn test_spdag_symetric() {

        let topo = gen_house();
        let spts = gen_house_spts();
        
        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::spdag(&topo, &(i+1));
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(expected_parents, parents);
            }
        }
    }

    #[test]
    fn test_spdag_asymetric() {

        let topo = gen_asymetric_house();
        let spts = gen_asymetric_house_spts();
       
        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::spdag(&topo, &(i+1));
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(expected_parents, parents);
            }
        }
    }

    #[test]
    fn test_revert_symetric() {
        let topo = gen_house();
        let reversed = DAG::revert(&topo);
        for (node, children) in topo {
            assert_eq!(children, reversed[&node]);
        }
    }

    #[test]
    fn test_revert_asymetric() {

        // 1 - (2) -> 3 
        // 3 - (6) -> 1
        let topo = gen_asymetric_house();

        // 1 - (6) -> 3 
        // 3 - (2) -> 1
        let mut expected = gen_house();
        expected.get_mut(&1).unwrap()[1].1 = 6;
        expected.get_mut(&3).unwrap()[0].1 = 2;

        let reversed = DAG::revert(&topo);
        //println!("{:#?}\n{:#?}", topo, reversed);
        for (node, children) in expected {
            assert_eq!(reversed[&node], children);
        }
    }

    #[test]
    fn test_rspdag_symetric() {
        let topo = gen_house();
        let spts = gen_house_spts();
       
        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::rspdag(&topo, &(i+1));
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(expected_parents, parents);
            }
        }
    }

     #[test]
    fn test_rspdag_asymetric() {

        let topo = gen_asymetric_house();
       
        let mut spts:Vec<HashMap<usize, Vec<usize>>> = Vec::with_capacity(6);
        spts.push(HashMap::from([
            (1, vec![1]),
            (2, vec![1]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![2]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3]),
            (3, vec![3]),
            (4, vec![2, 6]),
            (5, vec![3]),
            (6, vec![3])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![4]),
            (3, vec![2, 6]),
            (4, vec![4]),
            (5, vec![3]),
            (6, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3]),
            (3, vec![5]),
            (4, vec![2, 6]),
            (5, vec![5]),
            (6, vec![3])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3, 4]),
            (3, vec![6]),
            (4, vec![6]),
            (5, vec![3]),
            (6, vec![6])
        ]));

        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::rspdag(&topo, &(i+1));
            //println!("{}", i);
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(parents, expected_parents);
                /*for parent in parents {
                    assert!(&expected_parents.contains(&parent));
                }*/
            }
        }
    }

    #[test]
    fn test_depths_symetric() {

        // Setup
        let topo = gen_house();
        let expected = gen_symetric_spdags_max_depths();

        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let depths = DAGOrder::depths(&rspdag, root).0;
            expected[root].iter().for_each(|(node, depth)| assert_eq!(depths[node], *depth));
        }
    }

    #[test]
    fn test_depths_asymetric() {

        // Setup
        let topo = gen_asymetric_house();
        let expected = gen_symetric_spdags_max_depths();

        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let depths = DAGOrder::depths(&rspdag, root).0;
            expected[root].iter().for_each(|(node, depth)| assert_eq!(depths[node], *depth));
        }
    }

    #[test]
    fn test_order_symetric() {

        let topo = gen_house();
        let expected = gen_symetric_order();

        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let order = DAGOrder::order(&rspdag, root, 0);
            assert_eq!(order, expected[root]);
        }
    }

    #[test]
    fn order_on_failures() {
        let topo = gen_house();


        let rspdag = DAG::rspdag(&topo, &1);
        //println!("{:#?}", rspdag);

        // TODO: test all possiblities
        assert_eq!(DAG::order_failure(&topo, &2, &3, 666, 0), vec![5, 6]);
        assert_eq!(DAG::order_failure(&topo, &3, &2, 666, 0), vec![1, 4]);
    }
}

/// Module's tests on the house2floors topology
///
/// Each test comes in two flavours, one for the classical house topology (including ECMP paths)
/// and one for the house with an asymmetric link between nodes 1 and 3, i.e.:
/// 1 - (2) -> 3
/// 3 - (6) -> 1
/// This second topology allow cases where Node 1 can reach 3 via 3 and 2, but 3 can reach Node 1
/// only via Node 2.
///
/// TODO: Tests for parallel links between nodes.
///
#[cfg(test)]
mod house2floors {
    use crate::{WeightedHashSuccessors, DAG, HashSuccessors, graph::DAGOrder};
    use std::collections::HashMap;

    fn gen_house() -> WeightedHashSuccessors<usize, u16> {
        let mut topo: WeightedHashSuccessors<usize, u16> = WeightedHashSuccessors::new();
        topo.insert(1, vec![(2, 1), (3, 10)]);
        topo.insert(2, vec![(1, 1), (3, 1), (5, 10), (4, 1)]);
        topo.insert(3, vec![(1, 10), (2, 1), (5, 1), (6, 1)]);
        topo.insert(4, vec![(2, 1), (6, 1), (8, 1)]);
        topo.insert(5, vec![(2, 10), (3, 1)]);
        topo.insert(6, vec![(4, 1), (3, 1), (7, 1)]);
        topo.insert(7, vec![(6, 1), (8, 1)]);
        topo.insert(8, vec![(4, 1), (7, 1)]);

        topo
    }

    fn gen_asymetric_house() -> WeightedHashSuccessors<usize, u16> {
        // 1 - (2) -> 3
        // 3 - (6) -> 1
        let mut topo = gen_house();
        topo.get_mut(&1).unwrap()[1].1 = 2;
        topo.get_mut(&3).unwrap()[0].1 = 6;
        topo
    }

    fn gen_asymetric_house_spts() -> Vec<HashSuccessors<usize>> {
        let mut spts = gen_house_spts();
        let node1 = spts.get_mut(0).unwrap();
        let node1_3 = node1.get_mut(&3).unwrap();
        node1_3.push(1);
        node1_3.sort();
        spts
    }

    fn gen_house_spts() -> Vec<HashSuccessors<usize>> {
        let mut spts: Vec<HashSuccessors<usize>> = Vec::with_capacity(6);
        spts.push(HashMap::from([
            (1, vec![1]),
            (2, vec![1]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4]),
            (7, vec![6, 8]),
            (8, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![2]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4]),
            (7, vec![6, 8]),
            (8, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3]),
            (3, vec![3]),
            (4, vec![2, 6]),
            (5, vec![3]),
            (6, vec![3]),
            (7, vec![6]),
            (8, vec![4, 7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![4]),
            (3, vec![2, 6]),
            (4, vec![4]),
            (5, vec![3]),
            (6, vec![4]),
            (7, vec![6, 8]),
            (8, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3]),
            (3, vec![5]),
            (4, vec![2, 6]),
            (5, vec![5]),
            (6, vec![3]),
            (7, vec![6]),
            (8, vec![4, 7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3, 4]),
            (3, vec![6]),
            (4, vec![6]),
            (5, vec![3]),
            (6, vec![6]),
            (7, vec![6]),
            (8, vec![4, 7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![3, 4]),
            (3, vec![6]),
            (4, vec![6, 8]),
            (5, vec![3]),
            (6, vec![7]),
            (7, vec![7]),
            (8, vec![7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![4]),
            (3, vec![2, 6]),
            (4, vec![8]),
            (5, vec![3]),
            (6, vec![4, 7]),
            (7, vec![8]),
            (8, vec![8])
        ]));
        spts
    }

    fn gen_symetric_spdags_max_depths() -> HashMap<usize, HashMap<usize,usize>>{
        let mut expected: HashMap<usize, HashMap<usize,usize>> = HashMap::new();
        expected.insert(1, HashMap::from([(1,0), (2,1), (3,2), (4,2), (5,3), (6,3), (7,4), (8,3)]));
        expected.insert(2, HashMap::from([(1,1), (2,0), (3,1), (4,1), (5,2), (6,2), (7,3), (8,2)]));
        expected.insert(3, HashMap::from([(1,2), (2,1), (3,0), (4,2), (5,1), (6,1), (7,2), (8,3)]));
        expected.insert(4, HashMap::from([(1,2), (2,1), (3,2), (4,0), (5,3), (6,1), (7,2), (8,1)]));
        expected.insert(5, HashMap::from([(1,3), (2,2), (3,1), (4,3), (5,0), (6,2), (7,3), (8,4)]));
        expected.insert(6, HashMap::from([(1,3), (2,2), (3,1), (4,1), (5,2), (6,0), (7,1), (8,2)]));
        expected.insert(7, HashMap::from([(1,4), (2,3), (3,2), (4,2), (5,3), (6,1), (7,0), (8,1)]));
        expected.insert(8, HashMap::from([(1,3), (2,2), (3,3), (4,1), (5,4), (6,2), (7,1), (8,0)]));
        expected
    }

    fn gen_symetric_order() -> HashMap<usize, Vec<usize>> {
        let mut ret = HashMap::new();
        ret.insert(1, vec![7, 5, 6, 8, 3, 4, 2, 1]);
        ret.insert(2, vec![7, 5, 6, 8, 1, 3, 4, 2]);
        ret.insert(3, vec![8, 1, 4, 7, 2, 5, 6, 3]);
        ret.insert(4, vec![5, 1, 3, 7, 2, 6, 8, 4]);
        ret.insert(5, vec![8, 1, 4, 7, 2, 6, 3, 5]);
        ret.insert(6, vec![1, 2, 5, 8, 3, 4, 7, 6]);
        ret.insert(7, vec![1, 2, 5, 3, 4, 6, 8, 7]);
        ret.insert(8, vec![5, 1, 3, 2, 6, 4, 7, 8]);
        ret
    }

    fn _gen_asymetric_spdags_max_depths() -> HashMap<usize, HashMap<usize,usize>>{
        let mut expected: HashMap<usize, HashMap<usize,usize>> = HashMap::new();
        expected.insert(1, HashMap::from([(1,0), (2,1), (3,2), (4,2), (5,3), (6,3)]));
        expected.insert(2, HashMap::from([(1,1), (2,0), (3,1), (4,1), (5,2), (6,2)]));
        expected.insert(3, HashMap::from([(1,2), (2,1), (3,0), (4,2), (5,1), (6,1)]));
        expected.insert(4, HashMap::from([(1,2), (2,1), (3,2), (4,0), (5,3), (6,1)]));
        expected.insert(5, HashMap::from([(1,3), (2,2), (3,1), (4,3), (5,0), (6,2)]));
        expected.insert(6, HashMap::from([(1,3), (2,2), (3,1), (4,1), (5,2), (6,0)]));
        expected
    }

    #[test]
    fn test_spdag_symetric() {

        let topo = gen_house();
        let spts = gen_house_spts();
        
        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::spdag(&topo, &(i+1));
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                //println!("root <{}> parent<{}> got: {:?}, expected: {:?}",
                //         i+1, node, parents, expected_parents);
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(expected_parents, parents);
            }
        }
    }

    #[test]
    fn test_spdag_asymetric() {

        let topo = gen_asymetric_house();
        let spts = gen_asymetric_house_spts();
       
        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::spdag(&topo, &(i+1));
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(expected_parents, parents);
            }
        }
    }

    #[test]
    fn test_revert_symetric() {
        let topo = gen_house();
        let reversed = DAG::revert(&topo);
        for (node, children) in topo {
            assert_eq!(children, reversed[&node]);
        }
    }

    #[test]
    fn test_revert_asymetric() {

        // 1 - (2) -> 3 
        // 3 - (6) -> 1
        let topo = gen_asymetric_house();

        // 1 - (6) -> 3 
        // 3 - (2) -> 1
        let mut expected = gen_house();
        expected.get_mut(&1).unwrap()[1].1 = 6;
        expected.get_mut(&3).unwrap()[0].1 = 2;

        let reversed = DAG::revert(&topo);
        //println!("{:#?}\n{:#?}", topo, reversed);
        for (node, children) in expected {
            assert_eq!(reversed[&node], children);
        }
    }

    #[test]
    fn test_rspdag_symetric() {
        let topo = gen_house();
        let spts = gen_house_spts();
       
        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::rspdag(&topo, &(i+1));
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(expected_parents, parents);
            }
        }
    }

     #[test]
    fn test_rspdag_asymetric() {

        let topo = gen_asymetric_house();
       
        let mut spts:Vec<HashMap<usize, Vec<usize>>> = Vec::with_capacity(6);
        spts.push(HashMap::from([
            (1, vec![1]),
            (2, vec![1]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4]),
            (7, vec![6, 8]),
            (8, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![2]),
            (3, vec![2]),
            (4, vec![2]),
            (5, vec![3]),
            (6, vec![3, 4]),
            (7, vec![6, 8]),
            (8, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3]),
            (3, vec![3]),
            (4, vec![2, 6]),
            (5, vec![3]),
            (6, vec![3]),
            (7, vec![6]),
            (8, vec![4, 7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![4]),
            (3, vec![2, 6]),
            (4, vec![4]),
            (5, vec![3]),
            (6, vec![4]),
            (7, vec![6, 8]),
            (8, vec![4])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3]),
            (3, vec![5]),
            (4, vec![2, 6]),
            (5, vec![5]),
            (6, vec![3]),
            (7, vec![6]),
            (8, vec![4, 7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3, 4]),
            (3, vec![6]),
            (4, vec![6]),
            (5, vec![3]),
            (6, vec![6]),
            (7, vec![6]),
            (8, vec![4, 7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2, 3]),
            (2, vec![3, 4]),
            (3, vec![6]),
            (4, vec![6, 8]),
            (5, vec![3]),
            (6, vec![7]),
            (7, vec![7]),
            (8, vec![7])
        ]));
        spts.push(HashMap::from([
            (1, vec![2]),
            (2, vec![4]),
            (3, vec![2, 6]),
            (4, vec![8]),
            (5, vec![3]),
            (6, vec![4, 7]),
            (7, vec![8]),
            (8, vec![8])
        ]));

        for (i, _) in topo.iter().enumerate() {
            let spdag = DAG::rspdag(&topo, &(i+1));
            //println!("{}", i);
            for (node, parents) in spdag.unwrap().iter_mut() {
                let expected_parents = &(&spts)[i][&node];
                //println!("root <{}> parent<{}> got: {:?}, expected: {:?}",
                //         i+1, node, parents, expected_parents);

                // same number of parents
                assert_eq!(parents.len(), expected_parents.len());
                // check each parent
                parents.sort();
                assert_eq!(parents, expected_parents);
                /*for parent in parents {
                    assert!(&expected_parents.contains(&parent));
                }*/
            }
        }
    }


    #[test]
    fn test_depths_symetric() {

        // Setup
        let topo = gen_house();
        let expected = gen_symetric_spdags_max_depths();

        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let depths = DAGOrder::depths(&rspdag, root).0;
            expected[root].iter().for_each(|(node, depth)| assert_eq!(depths[node], *depth));
        }
    }

    #[test]
    fn test_depths_asymetric() {

        // Setup
        let topo = gen_asymetric_house();
        let expected = gen_symetric_spdags_max_depths();

        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let depths = DAGOrder::depths(&rspdag, root).0;
            expected[root].iter().for_each(|(node, depth)| assert_eq!(depths[node], *depth));
        }
    }


    #[test]
    fn test_order_symetric() {

        let topo = gen_house();
        let expected = gen_symetric_order();

        for (i, _) in topo.iter().enumerate() {
            let root = &(i+1);
            let rspdag = DAG::rspdag(&topo, root).unwrap();
            let order = DAGOrder::order(&rspdag, root, 0);
            assert_eq!(order, expected[root]);
            //println!("{:#?}", order);
        }
        //panic!();
    }

    #[test]
    fn order_on_failures() {
        let topo = gen_house();

        let rspdag = DAG::rspdag(&topo, &1);
        //println!("{:#?}", rspdag);

        // TODO: test all possibilites
        assert_eq!(DAG::order_failure(&topo, &2, &3, 666, 0), vec![7, 5, 6]);
        assert_eq!(DAG::order_failure(&topo, &3, &2, 666, 0), vec![8, 1, 4]);
    }
 
}

#[cfg(test)]
mod ring9 {
    use crate::*;
    use std::collections::HashMap;

    fn topo() -> WeightedHashSuccessors<usize, u16> {
        let mut topo: WeightedHashSuccessors<usize, u16> = WeightedHashSuccessors::new();
        topo.insert(1, vec![(9, 1), (2, 1)]);
        topo.insert(2, vec![(1, 1), (3, 1)]);
        topo.insert(3, vec![(2, 1), (4, 1)]);
        topo.insert(4, vec![(3, 1), (5, 1)]);
        topo.insert(5, vec![(4, 1), (6, 1)]);
        topo.insert(6, vec![(5, 1), (7, 1)]);
        topo.insert(7, vec![(6, 1), (8, 1)]);
        topo.insert(8, vec![(7, 1), (9, 1)]);
        topo.insert(9, vec![(8, 1), (1, 1)]);
        topo
    }

    
    #[test]
    fn test() {
        let topo = topo();

        let neighbor = 2;
        let root = 1;

        let order = DAG::order_failure(&topo, &root, &neighbor, 666, 0);        
        assert_eq!(order, vec![5, 4, 3]);

        let order = DAG::order_failure(&topo, &root, &neighbor, 666, 1);
        assert_eq!(order, vec![5, 4, 3]);

        //println!("order {:#?}", order);
        
        // assert!(false);
    }
}

#[cfg(test)]
mod grid9 {
    use crate::*;
    use std::collections::HashMap;

    fn gen_topo() -> WeightedHashSuccessors<usize, u16> {
        let mut topo: WeightedHashSuccessors<usize, u16> = WeightedHashSuccessors::new();
        topo.insert(1, vec![(2, 1), (4, 1)]);
        topo.insert(2, vec![(1, 1), (3, 1), (5, 1)]);
        topo.insert(3, vec![(2, 1), (6, 1)]);
        topo.insert(4, vec![(1, 1), (5, 1), (7, 1)]);
        topo.insert(5, vec![(2, 1), (4, 1), (6, 1), (8, 1)]);
        topo.insert(6, vec![(5, 1), (3, 1), (9, 1)]);
        topo.insert(7, vec![(4, 1), (8, 1)]);
        topo.insert(8, vec![(5, 1), (7, 1), (9, 1)]);
        topo.insert(9, vec![(6, 1), (8, 1)]);
        topo
    }

    #[test]
    fn test() {
        let topo = gen_topo();

        let current = 2;
        let root = 1;

        let order = DAG::order_failure(&topo, &root, &current, 666, 0);
        assert_eq!(order, vec![9, 6, 8, 3, 5]);

        let order = DAG::order_failure(&topo, &root, &current, 666, 1);
        assert_eq!(order, vec![6, 3, 5]);

        //println!("order {:#?}", order);
        
        // assert!(false);
    }
}

mod abilene {
    use crate::*;
    use std::collections::HashMap;

    static METRIC: u16 = 65534;

    fn gen_topo() -> WeightedHashSuccessors<usize, u16> {
        let mut topo: WeightedHashSuccessors<usize, u16> = WeightedHashSuccessors::new();
        topo.insert(1, vec![(2, 1), (3, 1)]);
        topo.insert(2, vec![(1, 1), (11, 1)]);
        topo.insert(3, vec![(1, 1), (10, 1)]);
        topo.insert(4, vec![(5, 1), (7, 1)]);
        topo.insert(5, vec![(4, 1), (6, 1), (7, 1)]);
        topo.insert(6, vec![(5, 1), (9, 1)]);
        topo.insert(7, vec![(4, 1), (5, 1), (8, 1)]);
        topo.insert(8, vec![(7, 1), (9, 1), (11, 1)]);
        topo.insert(9, vec![(6, 1), (8, 1), (10, 1)]);
        topo.insert(10, vec![(3, 1), (9, 1), (11, 1)]);
        topo.insert(11, vec![(2, 1), (8, 1), (10, 1)]);
        topo
    }

    #[test]
    fn test_8_7() {
        let topo = gen_topo();
        let current = 8;
        let peer = 7;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![10, 11, 9]);
    }

    #[test]
    fn test_7_8() {
        let topo = gen_topo();
        let current = 7;
        let peer = 8;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![5]);
    }
    
    #[test]
    fn test_9_6() {
        let topo = gen_topo();
        let current = 9;
        let peer = 6;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![11, 10, 8]);
    }    
   
    #[test]
    fn test_6_9() {
        let topo = gen_topo();
        let current = 6;
        let peer = 9;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![5]);
    }

    #[test]
    fn test_8_9() {
        let topo = gen_topo();
        let current = 8;
        let peer = 9;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![11]);
    }

    #[test]
    fn test_9_8() {
        let topo = gen_topo();
        let current = 9;
        let peer = 8;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![10]);
    }
    
    #[test]
    fn test_10_9() {
        let topo = gen_topo();
        let current = 10;
        let peer = 9;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![11]);
    }

    #[test]
    fn test_9_10() {
        
        let topo = gen_topo();
        let current = 9;
        let peer = 10;

        let order = DAG::order_failure(&topo, &peer, &current, METRIC, 1);
        assert_eq!(order, vec![8]);
    }


}
