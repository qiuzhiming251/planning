

#ifndef ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AGGREGATE_STATIC_OBSTACLE_COST_H_
#define ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AGGREGATE_STATIC_OBSTACLE_COST_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type
#include <algorithm>
#include <cmath>
#include <limits>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "plan_common/log.h"

//#include "global/buffered_logger.h"
//#include "lite/check.h"
#include "plan_common/math/geometry/segment2d.h"
#include "plan_common/math/segment_matcher/segment_matcher_kdtree.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "planner/trajectory_optimizer/problem/cost.h"

namespace st {
namespace planning {

template <typename PROB>
class AggregateStaticObjectCost : public Cost<PROB> {
 public:
  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;

  using DividedG = typename Cost<PROB>::DividedG;

  using CostType = typename Cost<PROB>::CostType;

  using PenetrationJacobianType = Eigen::Matrix<double, 1, PROB::kStateSize>;
  using PenetrationHessianType =
      Eigen::Matrix<double, PROB::kStateSize, PROB::kStateSize>;

  static constexpr double kCutoffDistance = 50.0;
  static constexpr double kNormalizedScale = 1.0;
  AggregateStaticObjectCost(
      const std::vector<double>& extra_gain_for_nudge,
      const std::vector<Segment2d>& segments, std::vector<double> dist_to_rac,
      std::vector<double> angle_to_axis,
      std::vector<std::vector<double>> buffers, std::vector<double> gains,
      std::vector<std::string> sub_names, int num_objects,
      bool using_hessian_approximate,
      std::string name = absl::StrCat(PROB::kProblemPrefix,
                                      "AggregateStaticObjectCost"),
      double scale = 1.0,
      CostType cost_type = Cost<PROB>::CostType::GROUP_OBJECT)
      : Cost<PROB>(std::move(name), scale * kNormalizedScale, cost_type),
        static_object_boundary_(segments),
        num_objects_(num_objects),
        dist_to_rac_(std::move(dist_to_rac)),
        angle_to_axis_(std::move(angle_to_axis)),
        circle_size_(dist_to_rac_.size()),
        buffers_(std::move(buffers)),
        gains_(std::move(gains)),
        sub_names_(std::move(sub_names)),
        using_hessian_approximate_(using_hessian_approximate) {
    CHECK_EQ(sub_names_.size(), gains_.size());
    CHECK_EQ(dist_to_rac_.size(), angle_to_axis_.size());
    CHECK_EQ(buffers_.size(), dist_to_rac_.size());
    for (const auto& buffer : buffers_) {
      CHECK_EQ(buffer.size(), sub_names_.size());
    }

    penetrations_.resize(num_objects_);
    segments_ptr_.resize(num_objects_);
    penetration_jacobians_.resize(num_objects_);
    penetration_hessians_.resize(num_objects_);
    for (int k = 0; k < num_objects_; ++k) {
      penetrations_[k].resize(circle_size_);
      penetration_jacobians_[k].resize(circle_size_,
                                       PenetrationJacobianType::Zero());
      penetration_hessians_[k].resize(circle_size_,
                                      PenetrationHessianType::Zero());
      segments_ptr_[k].resize(circle_size_, nullptr);
    }

    for (int i = 0, n = num_objects_; i < n; ++i) {
      for (int idx = 0; idx < circle_size_; ++idx) {
        auto& penetration = penetrations_[i][idx];
        penetration.reserve(gains_.size());
        for (int k = 0; k < gains_.size(); ++k) {
          penetration.push_back(std::numeric_limits<double>::infinity());
        }
      }
    }

    extra_gain_for_nudge_ = extra_gain_for_nudge;
  }

  const std::vector<std::vector<std::vector<double>>>& penetrations() const {
    return penetrations_;
  }

  DividedG SumGForAllSteps(const StatesType& xs, const ControlsType& us,
                           int horizon) const override {
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
      res.SetIsSoft(i, sub_names_[i] == "a");
    }
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      for (int idx = 0; idx < circle_size_; ++idx) {
        const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
        for (int i = 0; i < penetrations_k.size(); ++i) {
          if (penetrations_k[i] < 0.0) {
            res.AddSubG(i, extra_gain_for_nudge_[k] * gains_[i] *
                               Sqr(penetrations_k[i]));
          }
        }
      }
    }
    res.Multi(0.5 * Cost<PROB>::scale());
    return res;
  }

  // g.
  DividedG EvaluateGWithDebugInfo(int k, const StateType& x,
                                  const ControlType& u,
                                  bool using_scale) const override {
    DCHECK_GE(k, 0);
    std::vector<double> gains(gains_.size());
    if (using_scale) {
      gains = gains_;
    } else {
      std::fill(gains.begin(), gains.end(), 1.0);
    }
    DividedG res(sub_names_.size());
    for (int i = 0; i < sub_names_.size(); ++i) {
      res.SetSubName(i, sub_names_[i] + Cost<PROB>::name());
    }
    if (k >= num_objects_) return res;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          res.AddSubG(i, 0.5 * extra_gain_for_nudge_[k] * Cost<PROB>::scale() *
                             gains[i] * Sqr(penetrations_k[i]));
        }
      }
    }
    return res;
  }

  // g.
  double EvaluateG(int k, const StateType& x,
                   const ControlType& u) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return 0.0;
    double g = 0.0;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          g += 0.5 * extra_gain_for_nudge_[k] * Cost<PROB>::scale() *
               gains_[i] * Sqr(penetrations_k[i]);
        }
      }
    }
    return g;
  }

  // Gradients with superposition.
  void AddDGDx(int k, const StateType& x, const ControlType& u,
               DGDxType* dgdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      const auto& penetration_jacobians_k = penetration_jacobians_[k][idx];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          *dgdx += extra_gain_for_nudge_[k] * Cost<PROB>::scale() * gains_[i] *
                   penetrations_k[i] * penetration_jacobians_k;
        }
      }
    }
  }
  void AddDGDu(int k, const StateType& x, const ControlType& u,
               DGDuType* dgdu) const override {}

  // Hessians with superposition.
  void AddDDGDxDx(int k, const StateType& x, const ControlType& u,
                  DDGDxDxType* ddgdxdx) const override {
    DCHECK_GE(k, 0);
    if (k >= num_objects_) return;
    for (int idx = 0; idx < circle_size_; ++idx) {
      const auto penetrations_k = absl::MakeConstSpan(penetrations_[k][idx]);
      const auto& penetration_jacobians_k = penetration_jacobians_[k][idx];
      const auto& penetration_hessians_k = penetration_hessians_[k][idx];
      for (int i = 0; i < penetrations_k.size(); ++i) {
        if (penetrations_k[i] < 0.0) {
          if (using_hessian_approximate_) {
            *ddgdxdx +=
                extra_gain_for_nudge_[k] * Cost<PROB>::scale() * gains_[i] *
                (penetration_jacobians_k.transpose() * penetration_jacobians_k);
          } else {
            *ddgdxdx +=
                extra_gain_for_nudge_[k] * Cost<PROB>::scale() * gains_[i] *
                (penetrations_k[i] * penetration_hessians_k +
                 penetration_jacobians_k.transpose() * penetration_jacobians_k);
          }
        }
      }
    }
  }
  void AddDDGDuDx(int k, const StateType& x, const ControlType& u,
                  DDGDuDxType* ddgdudx) const override {}
  void AddDDGDuDu(int k, const StateType& x, const ControlType& u,
                  DDGDuDuType* ddgdudu) const override {}

  void Update(const StatesType& xs, const ControlsType& /*us*/,
              int horizon) override {
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      for (int idx = 0; idx < circle_size_; ++idx) {
        auto penetrations_k = absl::MakeSpan(penetrations_[k][idx]);
        const auto x0 = PROB::GetStateAtStep(xs, k);
        // Vehicle position, rear center.
        const Vec2d pos = PROB::pos(xs, k);
        const double dist_to_rac = dist_to_rac_[idx];
        const double angle_to_axis = angle_to_axis_[idx];
        const std::vector<double>& buffers = buffers_[idx];
        const Vec2d tangent = Vec2d::FastUnitFromAngleN12(
            PROB::StateGetTheta(x0) + angle_to_axis);
        const double xr = pos.x() + dist_to_rac * tangent.x();
        const double yr = pos.y() + dist_to_rac * tangent.y();
        const Vec2d pt = {xr, yr};
        const Segment2d* line_pt =
            static_object_boundary_.GetNearestSegment(xr, yr);
        segments_ptr_[k][idx] = line_pt;
        double msd = kCutoffDistance;
        if (line_pt != nullptr) {
          msd = line_pt->DistanceTo(pt);
        }
        for (size_t i = 0; i < penetrations_k.size(); ++i) {
          penetrations_k[i] = msd - buffers[i];
        }
      }
    }
  }

  void UpdateDerivatives(const StatesType& xs, const ControlsType& /*us*/,
                         int horizon) override {
    horizon = std::min(horizon, num_objects_);
    for (int k = 0; k < horizon; ++k) {
      for (int idx = 0; idx < circle_size_; ++idx) {
        auto& penetration_jacobians_k = penetration_jacobians_[k][idx];
        auto& penetration_hessians_k = penetration_hessians_[k][idx];

        penetration_jacobians_k.template segment<3>(PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 1>::Zero();
        penetration_hessians_k.template block<3, 3>(PROB::kStateXIndex,
                                                    PROB::kStateXIndex) =
            Eigen::Matrix<double, 3, 3>::Zero();
        const auto x0 = PROB::GetStateAtStep(xs, k);
        const double dist_to_rac = dist_to_rac_[idx];
        const double angle_to_axis = angle_to_axis_[idx];

        const Vec2d tangent = Vec2d::FastUnitFromAngleN12(
            PROB::StateGetTheta(x0) + angle_to_axis);
        VLOG(3) << "Step " << k;
        VLOG(4) << "x0 = " << x0.transpose()
                << " tangent = " << tangent.transpose();
        // Vehicle position, rear center.
        const Vec2d pos = PROB::pos(xs, k);
        const Segment2d* line_pt = segments_ptr_[k][idx];
        const double xr = pos.x() + dist_to_rac * tangent.x();
        const double yr = pos.y() + dist_to_rac * tangent.y();
        const Vec2d pt = {xr, yr};
        const double projection = line_pt->ProjectOntoUnit(pt);
        const Vec2d normal = tangent.Perp();
        if (projection >= 0.0 && projection <= line_pt->length()) {
          const double signed_dist = line_pt->ProductOntoUnit(pt);
          const Vec2d force_dir =
              (signed_dist < 0.0 ? -line_pt->unit_direction().Perp()
                                 : line_pt->unit_direction().Perp());
          penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
              force_dir;
          penetration_jacobians_k(PROB::kStateThetaIndex) =
              force_dir.dot(normal * dist_to_rac);
          if (!using_hessian_approximate_) {
            penetration_hessians_k(PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
                force_dir.dot(-tangent * dist_to_rac);
          }
        } else {
          const auto& pt2 =
              projection < 0.0 ? line_pt->start() : line_pt->end();
          const Vec2d x1 = pt - pt2;
          const double dist = std::sqrt(x1.Sqr());
          const double dist_inv = 1.0 / dist;
          const double dist_inv_cube = Cube(dist_inv);
          penetration_jacobians_k.template segment<2>(PROB::kStateXIndex) =
              x1 * dist_inv;
          penetration_jacobians_k(PROB::kStateThetaIndex) =
              x1.dot(normal * dist_to_rac) * dist_inv;
          if (!using_hessian_approximate_) {
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateXIndex) =
                dist_inv - Sqr(x1.x()) * dist_inv_cube;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex) =
                -x1.x() * x1.y() * dist_inv_cube;
            penetration_hessians_k(PROB::kStateXIndex, PROB::kStateThetaIndex) =
                normal.x() * dist_to_rac * dist_inv -
                dist_inv_cube * x1.x() * x1.dot(normal * dist_to_rac);

            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex, PROB::kStateYIndex);
            penetration_hessians_k(PROB::kStateYIndex, PROB::kStateYIndex) =
                dist_inv - Sqr(x1.y()) * dist_inv_cube;
            penetration_hessians_k(1, PROB::kStateThetaIndex) =
                normal.y() * dist_to_rac * dist_inv -
                dist_inv_cube * x1.y() * x1.dot(normal * dist_to_rac);

            penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateXIndex) =
                penetration_hessians_k(PROB::kStateXIndex,
                                       PROB::kStateThetaIndex);
            penetration_hessians_k(PROB::kStateThetaIndex, PROB::kStateYIndex) =
                penetration_hessians_k(PROB::kStateYIndex,
                                       PROB::kStateThetaIndex);
            penetration_hessians_k(PROB::kStateThetaIndex,
                                   PROB::kStateThetaIndex) =
                (x1.dot(-tangent * dist_to_rac) +
                 dist_to_rac * normal.dot(normal * dist_to_rac)) *
                    dist_inv -
                dist_inv_cube * x1.dot(normal * dist_to_rac) *
                    x1.dot(normal * dist_to_rac);
          }
        }
      }
    }
  }

 private:
  SegmentMatcherKdtree static_object_boundary_;
  int num_objects_;

  // Distances from control points to RAC on vehicle longitudinal axis.
  std::vector<double> dist_to_rac_;
  std::vector<double> angle_to_axis_;
  int circle_size_;

  // Buffer to static object
  std::vector<std::vector<double>> buffers_;
  std::vector<double> gains_;
  std::vector<std::string> sub_names_;
  bool using_hessian_approximate_;

  // States.
  std::vector<std::vector<std::vector<double>>> penetrations_;
  std::vector<std::vector<const Segment2d*>> segments_ptr_;
  std::vector<std::vector<PenetrationJacobianType>> penetration_jacobians_;
  std::vector<std::vector<PenetrationHessianType>> penetration_hessians_;
  std::vector<double> extra_gain_for_nudge_;
};

}  // namespace planning
}  // namespace st

// NOLINTNEXTLINE
#endif  // ONBOARD_PLANNER_OPTIMIZATION_PROBLEM_AGGREGATE_STATIC_OBSTACLE_COST_H_
