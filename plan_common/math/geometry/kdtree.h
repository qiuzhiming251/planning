

#ifndef ONBOARD_MATH_GEOMETRY_KDTREE_H_
#define ONBOARD_MATH_GEOMETRY_KDTREE_H_

#include <cstddef>
#include <memory>  // IWYU pragma: keep
#include <set>
#include <vector>

#include "Eigen/Core"                      // IWYU pragma: keep
#include "plan_common/math/point_types.h"  // // IWYU pragma: keep
#include "plan_common/math/vec.h"          // IWYU pragma: keep

namespace st {

// KD tree that is used to search nearest neighbors.
// Only support point types from Eigen fixed length vectors such as Vec3d.
template <typename PointType>
class KDTree {
 public:
  using ScalarType = typename PointType::Scalar;

  KDTree() = default;

  explicit KDTree(const std::vector<PointType>& points);

  KDTree(const std::vector<PointType>& points,
         const std::vector<int>& valid_indexes);

  const PointType& FindNearest(const PointType& point,
                               ScalarType* best_dist_sqr = nullptr) const;

  std::set<int> FindKNearest(const PointType& point, int k) const;

  std::set<int> FindNearestInRadius(const PointType& point,
                                    const ScalarType max_dist_sqr) const;

  const PointType& PointAt(int index) const { return nodes_[index].point; }

  // Index return by nearest queries are shuffled tree index, which can be
  // converted to original index for relational data look up
  int GetOriginalIndex(int tree_index) const {
    return nodes_[tree_index].original_index;
  }

  std::set<int> GetOriginalIndices(const std::set<int>& tree_indices) const;

  size_t size() const { return nodes_.size(); }

 private:
  struct TreeNode {
    PointType point;
    int left;
    int right;
    int original_index;
  };

  using NodeIter = typename std::vector<TreeNode>::iterator;
  static constexpr int kDim = PointType::RowsAtCompileTime;

  // Sort nodes such that all nodes in [begin, mid) have values (at dim) less
  // than the node at mid, and all nodes in (mid, end) have values (at dim)
  // larger than the node at mid.
  int FindMedian(int begin, int end, int dim);

  // Recursively make a KD tree.
  int MakeTree(int begin, int end, int dim);

  template <bool kIgnoreIndicesInSet>
  void FindNearest(const PointType& point, int node_index, int dim,
                   const std::set<int>& to_ignore, int* best,
                   ScalarType* best_dist_sqr) const;

  // The node index of the root node.
  int root_index_ = -1;

  // All tree nodes.
  std::vector<TreeNode> nodes_;
};

}  // namespace st

#endif  // ONBOARD_MATH_GEOMETRY_KDTREE_H_
