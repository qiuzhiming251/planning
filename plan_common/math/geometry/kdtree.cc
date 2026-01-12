

#include <algorithm>
#include <limits>

#include "Eigen/Core"
#include "plan_common/math/geometry/kdtree.h"
#include "plan_common/math/point_types.h"
#include "plan_common/math/util.h"

namespace st {

template <typename PointType>
KDTree<PointType>::KDTree(const std::vector<PointType>& points) {
  nodes_.resize(points.size());
  for (int i = 0; i < points.size(); ++i) {
    nodes_[i].point = points[i];
    nodes_[i].original_index = i;
  }
  root_index_ = MakeTree(0, nodes_.size(), 0);
}

template <typename PointType>
KDTree<PointType>::KDTree(const std::vector<PointType>& points,
                          const std::vector<int>& valid_indexes) {
  nodes_.reserve(valid_indexes.size());
  for (const auto valid : valid_indexes) {
    if (valid < 0 || valid >= points.size()) continue;
    nodes_.push_back({});
    nodes_.back().point = points[valid];
    nodes_.back().original_index = valid;
  }
  root_index_ = MakeTree(0, nodes_.size(), 0);
}

template <typename PointType>
const PointType& KDTree<PointType>::FindNearest(
    const PointType& point, ScalarType* best_dist_sqr) const {
  ScalarType best_dist_sqr_local = std::numeric_limits<ScalarType>::max();
  int nearest_index;
  FindNearest<false>(point, root_index_, 0, {}, &nearest_index,
                     &best_dist_sqr_local);
  if (best_dist_sqr != nullptr) {
    *best_dist_sqr = best_dist_sqr_local;
  }
  return nodes_[nearest_index].point;
}

template <typename PointType>
std::set<int> KDTree<PointType>::FindKNearest(const PointType& point,
                                              int k) const {
  CHECK_LE(k, nodes_.size());
  std::set<int> nearest_indices;
  for (int i = 0; i < k; ++i) {
    int nearest_index;
    ScalarType best_dist_sqr = std::numeric_limits<ScalarType>::max();
    FindNearest<true>(point, root_index_, 0, nearest_indices, &nearest_index,
                      &best_dist_sqr);
    nearest_indices.insert(nearest_index);
  }
  return nearest_indices;
}

template <typename PointType>
std::set<int> KDTree<PointType>::FindNearestInRadius(
    const PointType& point, const ScalarType max_dist_sqr) const {
  std::set<int> nearest_indices;
  while (max_dist_sqr > 0) {
    int nearest_index;
    ScalarType best_dist_sqr = std::numeric_limits<ScalarType>::max();
    FindNearest<true>(point, root_index_, 0, nearest_indices, &nearest_index,
                      &best_dist_sqr);
    if (best_dist_sqr > max_dist_sqr) break;
    nearest_indices.insert(nearest_index);
  }
  return nearest_indices;
}

template <typename PointType>
std::set<int> KDTree<PointType>::GetOriginalIndices(
    const std::set<int>& tree_indices) const {
  std::set<int> original_indices;
  for (int tree_index : tree_indices) {
    original_indices.insert(nodes_[tree_index].original_index);
  }
  return original_indices;
}

template <typename PointType>
int KDTree<PointType>::FindMedian(int begin, int end, int dim) {
  if (end == begin) return -1;
  if (end == begin + 1) return begin;

  const auto mid = begin + (end - begin) / 2;
  std::nth_element(nodes_.begin() + begin, nodes_.begin() + mid,
                   nodes_.begin() + end,
                   [dim](const TreeNode& a, const TreeNode& b) {
                     return a.point[dim] < b.point[dim];
                   });
  return mid;
}

template <typename PointType>
int KDTree<PointType>::MakeTree(int begin, int end, int dim) {
  const auto mid = FindMedian(begin, end, dim);
  if (mid >= 0) {
    if (++dim == kDim) dim = 0;
    nodes_[mid].left = MakeTree(begin, mid, dim);
    nodes_[mid].right = MakeTree(mid + 1, end, dim);
  }
  return mid;
}

template <typename PointType>
template <bool IgnoreIndicesInSet>
void KDTree<PointType>::FindNearest(const PointType& point, int node_index,
                                    int dim, const std::set<int>& to_ignore,
                                    int* best,
                                    ScalarType* best_dist_sqr) const {
  struct State {
    int node_index;
    int dim;
    ScalarType min_dist2;
  };
  std::vector<State> stack;
  stack.reserve(64);
  stack.push_back({node_index, dim, 0.0});

  ScalarType best_dist2 = std::numeric_limits<ScalarType>::max();
  int best_index{-1};

  while (!stack.empty()) {
    const auto [index, this_dim, min_dist2] = stack.back();
    stack.pop_back();

    // The best distance is already better than any point in the other side.
    if (best_dist2 <= min_dist2) continue;

    const auto& node = nodes_[index];
    if (!(IgnoreIndicesInSet && to_ignore.count(index))) {
      const ScalarType dist2 = (node.point - point).squaredNorm();
      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best_index = index;
      }
    }

    const ScalarType dx = point(this_dim) - node.point(this_dim);
    const int likely_index = dx < 0 ? node.left : node.right;
    const int unlikely_index = dx < 0 ? node.right : node.left;

    int next_dim = this_dim + 1;
    if (next_dim == kDim) next_dim = 0;

    if (unlikely_index >= 0) {
      stack.push_back({unlikely_index, next_dim, Sqr(dx)});
    }
    if (likely_index >= 0) {
      stack.push_back({likely_index, next_dim, 0.0});
    }
  }

  *best_dist_sqr = best_dist2;
  *best = best_index;
}

template class KDTree<Vec3d>;
template class KDTree<Vec3f>;
template class KDTree<Vec2d>;
template class KDTree<Vec2f>;
using namespace point_types;  // NOLINT
template class KDTree<PointXYZI>;
template class KDTree<PointXYIdx>;
}  // namespace st
