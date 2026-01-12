

#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plan_common/log.h"
#include "plan_common/math/eigen.h"
#include "plan_common/math/frenet_frame.h"
#include "planner/trajectory_optimizer/problem/cost_helper.h"

namespace st {
namespace planning {

template <typename PROB>
class CenterLineQueryHelper : public CostHelper<PROB> {
 public:
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  CenterLineQueryHelper(int horizon, const std::vector<Vec2d>& points,
                        int last_real_point_index, std::string name,
                        bool use_qtfm)
      : CostHelper<PROB>(horizon, std::move(name)),
        last_real_point_index_(last_real_point_index) {
    CHECK_GT(points.size(), 1);
    constexpr double kEps = 1e-9;
    for (int idx = 0; idx < points.size() - 1; ++idx) {
      const double dist2 = (points[idx + 1] - points[idx]).squaredNorm();
      CHECK_GE(dist2, kEps);
    }
    if (use_qtfm) {
      path_ = std::make_unique<QtfmEnhancedKdTreeFrenetFrame>(
          BuildQtfmEnhancedKdTreeFrenetFrame(points,
                                             /*down_sample_raw_points=*/false)
              .value());
    } else {
      path_ = std::make_unique<KdTreeFrenetFrame>(
          BuildKdTreeFrenetFrame(points, /*down_sample_raw_points=*/false)
              .value());
    }
    // Make sure frenet frame has same path points with ref_points, o we don't
    // down sample ref points when building frenet frame.
    CHECK_EQ(points.size(), path_->points().size());

    // Initialize the 5 vectors with length CostHelper<PROB>::horizon().
    s_l_list_.resize(CostHelper<PROB>::horizon());
    normals_.resize(CostHelper<PROB>::horizon());
    indices_.resize(CostHelper<PROB>::horizon());
    index_pairs_.resize(CostHelper<PROB>::horizon());
    alphas_.resize(CostHelper<PROB>::horizon());
  }

  const std::vector<FrenetCoordinate>& s_l_list() const { return s_l_list_; }
  const std::vector<Vec2d>& normals() const { return normals_; }
  const std::vector<int>& indices() const { return indices_; }
  const std::vector<std::pair<int, int>>& index_pairs() const {
    return index_pairs_;
  }
  const std::vector<double>& alphas() const { return alphas_; }
  const std::vector<double>& s_knots() const { return path_->s_knots(); }
  const std::vector<Vec2d>& tangents() const { return path_->tangents(); }
  const std::vector<Vec2d>& points() const { return path_->points(); }
  int last_real_point_index() const { return last_real_point_index_; }

  void Update(const StatesType& xs, const ControlsType& us) override {
    const auto& raw_indices = path_->raw_indices();
    for (int k = 0; k < CostHelper<PROB>::horizon(); ++k) {
      const Vec2d pos = PROB::StateGetPos(PROB::GetStateAtStep(xs, k));
      int* index_ptr = &indices_[k];
      path_->XYToSL(pos, &s_l_list_[k], &normals_[k], index_ptr, &alphas_[k]);
      auto& index_pair = index_pairs_[k];
      index_pair.first = raw_indices[*index_ptr];
      index_pair.second = raw_indices[(*index_ptr) + 1];
    }
  }

  const FrenetFrame& path() const {
    CHECK_NOTNULL(path_);
    return *path_;
  }

 private:
  std::string name_ = "";
  std::unique_ptr<FrenetFrame> path_;
  int last_real_point_index_;

  // Frenet xy to sl query results.
  std::vector<FrenetCoordinate> s_l_list_;
  std::vector<Vec2d> normals_;
  std::vector<int> indices_;
  std::vector<std::pair<int, int>> index_pairs_;
  std::vector<double> alphas_;
};

}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_CENTER_LINE_QUERY_HELPER_H_
