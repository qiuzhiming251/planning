

#ifndef AD_BYD_PLANNING_UTILS_KDTREE_H
#define AD_BYD_PLANNING_UTILS_KDTREE_H

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace ad_byd {
namespace planning {

template <typename coordinate_type, size_t dimensions, typename T>
class KDPoint {
 public:
  KDPoint(std::array<coordinate_type, dimensions> c, T val)
      : _coords(c), _val(val) {}
  KDPoint(std::array<coordinate_type, dimensions> c) : _coords(c), _val() {}

  coordinate_type Get(size_t index) const { return _coords[index]; }

  T GetValue() const { return _val; }

  double Distance(const KDPoint &pt) const {
    double dist = 0;
    for (size_t i = 0; i < dimensions; ++i) {
      double d = Get(i) - pt.Get(i);
      dist += d * d;
    }
    return dist;
  }

 private:
  std::array<coordinate_type, dimensions> _coords;
  T _val;
};

template <typename coordinate_type, size_t dimensions, typename T>
class KDTree {
 public:
  typedef KDPoint<coordinate_type, dimensions, T> point_type;

 public:
  KDTree(const KDTree &) = delete;
  KDTree &operator=(const KDTree &) = delete;

  KDTree() = default;
  ~KDTree() = default;

  template <typename iterator>
  KDTree(iterator begin, iterator end) {
    _nodes.reserve(std::distance(begin, end));
    for (auto i = begin; i != end; ++i) {
      _nodes.emplace_back(*i);
    }
    _root = MakeTree(0, _nodes.size(), 0);
  }

  template <typename iterator>
  void Update(iterator begin, iterator end) {
    _nodes.clear();
    _nodes.reserve(std::distance(begin, end));
    for (auto i = begin; i != end; ++i) {
      _nodes.emplace_back(*i);
    }
    _root = MakeTree(0, _nodes.size(), 0);
  }

  bool empty() const { return _nodes.empty(); }

  void NearPoint(const point_type &pt, const double range,
                 std::vector<point_type> *const points) const {
    NearPoint(_root, pt, 0, range, points);
  }

 private:
  struct node {
    node(const point_type &pt) : _point(pt), _left(nullptr), _right(nullptr) {}

    coordinate_type Get(size_t index) const { return _point.Get(index); }

    double Distance(const point_type &pt) const { return _point.Distance(pt); }

    point_type _point;
    node *_left;
    node *_right;
  };

  struct node_cmp {
    node_cmp(size_t index) : _index(index) {}

    bool operator()(const node &n1, const node &n2) const {
      return n1._point.Get(_index) < n2._point.Get(_index);
    }

    size_t _index;
  };

  node *MakeTree(size_t begin, size_t end, size_t index) {
    if (end <= begin) {
      return nullptr;
    }

    size_t n = begin + (end - begin) / 2;
    std::nth_element(&_nodes[begin], &_nodes[n], &_nodes[end], node_cmp(index));
    index = (index + 1) % dimensions;
    _nodes[n]._left = MakeTree(begin, n, index);
    _nodes[n]._right = MakeTree(n + 1, end, index);
    return &_nodes[n];
  }

  void NearPoint(node *root, const point_type &point, size_t index,
                 const double range,
                 std::vector<point_type> *const points) const {
    if (root == nullptr) {
      return;
    }

    double d = root->Distance(point);
    if (d < range * range) {
      points->push_back(root->_point);
    }
    if (std::fabs(range) < 1e-6) {
      return;
    }

    double dx = root->Get(index) - point.Get(index);
    index = (index + 1) % dimensions;

    NearPoint(dx > 0 ? root->_left : root->_right, point, index, range, points);
    if (dx >= range) {
      return;
    }
    NearPoint(dx > 0 ? root->_right : root->_left, point, index, range, points);
  }

 private:
  node *_root = nullptr;
  std::vector<node> _nodes;
};

}  // namespace planning
}  // namespace ad_byd

#endif  // AD_BYD_PLANNING_UTILS_KDTREE_H
