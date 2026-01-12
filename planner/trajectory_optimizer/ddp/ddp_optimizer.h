

#ifndef ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_
#define ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_

// IWYU pragma: no_include <ext/alloc_traits.h>
//              for __alloc_traits<>::value_type

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <numeric>
#include <optional>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"  // IWYU pragma: keep
#include "Eigen/Eigenvalues"
#include "Eigen/LU"  // IWYU pragma: keep
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "plan_common/base/macros.h"
#include "plan_common/log_data.h"
#include "gflags/gflags.h"
#include "plan_common/log.h"
//#include "global/trace.h"
//#include "lite/check.h"
#include "plan_common/math/util.h"
#include "plan_common/math/vec.h"
#include "modules/cnoa_pnc/planning/proto/planner_params.pb.h"
#include "ddp_cost_manager_monitor.h"
#include "ddp_optimizer_monitor.h"
#include "planner/trajectory_optimizer/problem/cost.h"
#include "planner/planner_manager/planner_defs.h"
#include "plan_common/trajectory_point.h"

namespace st::planning {
template <typename PROB>

class DdpOptimizer;
}  // namespace st::planning

namespace st {
namespace planning {

#define DDPVLOG(verboselevel) \
  VLOG_IF(verboselevel, ((verbosity_ >= verboselevel)))

// Type PROB is the DDP problem definition.
// A DDP problem is a tuple (f, g^n) where f is the (non-linear) system dynamics
// and g^n is the (time-variable) cost, both of which functions of x and u.
template <typename PROB>
class DdpOptimizer {
 public:
  // Configuration for solve action.
  struct SolveConfig {
    int max_iteration = INT_MAX;
    bool forward = true;
    bool enable_iteration_failure_postprocess = true;

    static constexpr SolveConfig Default() {
      return SolveConfig{
          .max_iteration = INT_MAX,
          .forward = true,
          .enable_iteration_failure_postprocess = true,
      };
    }

    static constexpr SolveConfig Onboard() {
      return SolveConfig{
          .max_iteration = INT_MAX,
          .forward = true,
          .enable_iteration_failure_postprocess = true,
      };
    }
  };

  static constexpr int kStateSize = PROB::kStateSize;
  static constexpr int kControlSize = PROB::kControlSize;

  using StateType = typename PROB::StateType;
  using ControlType = typename PROB::ControlType;
  using StatesType = typename PROB::StatesType;
  using ControlsType = typename PROB::ControlsType;

  // problem will not be owned.
  // But member of problem will be changed when calling
  // const member functions of DdpOptimizer.
  DdpOptimizer(int plan_id, const PROB* problem, int horizon, std::string owner,
               int verbosity, DdpOptimizerParamsProto params,
               std::string dopt_tag);

  DdpOptimizer(int plan_id, const PROB* problem, int horizon, std::string owner,
               int verbosity, DdpOptimizerParamsProto params)
      : DdpOptimizer(plan_id, problem, horizon, owner, verbosity, params,
                     /*dopt_tag=*/"0") {}

  DdpOptimizer(int plan_id, const PROB* problem, int horizon, std::string owner,
               int verbosity)
      : DdpOptimizer(plan_id, problem, horizon, owner, verbosity,
                     /*params=*/CreateDefaultParams()) {}

  DdpOptimizer(int plan_id, const PROB* problem, int horizon, std::string owner)
      : DdpOptimizer(plan_id, problem, horizon, owner, /*verbosity=*/0) {}

  DdpOptimizer(int plan_id, const PROB* problem, int horizon)
      : DdpOptimizer(plan_id, problem, horizon, /*owner=*/"") {}

  using MonitorType = typename st::planning::DdpOptimizerMonitor<PROB>;

  // Not owned.
  void AddMonitor(MonitorType* monitor) { monitors_.push_back(monitor); }

  // Optimize the given init_trajectory, making its cost smaller.
  // Pre-requisite:
  // init_trajectory.size() must greater equal to horizon_.
  absl::StatusOr<std::vector<TrajectoryPoint>> Solve(
      const std::vector<TrajectoryPoint>& init_trajectory,
      std::vector<std::string>* final_cost_debug_info, double* const final_cost,
      const SolveConfig& config = SolveConfig::Default(),
      bool is_compare_weight = false) const;

  // Evaluate the cost of the given trajectory.
  // Pre-requisite:
  // init_trajectory.size() must greater equal to horizon_.
  double EvaluateCostForTrajectory(
      const std::vector<TrajectoryPoint>& trajectory) const {
    CHECK_GE(trajectory.size(), horizon_);
    StateType x0;
    ControlsType us(horizon_ * kControlSize);
    StatesType xs(horizon_ * kStateSize);
    FitTrajectoryPointToSolverStates(trajectory, &x0, &us, &xs);

    typename MonitorType::OptimizerInspector oi;
    for (auto* monitor : monitors_) {
      monitor->OnSolveStart(xs, us, oi);
    }

    return EvaluateCost(xs, us);
  }

  // This function will compute the accumulative discounted(based on gamma) cost
  // for different cost type, and store them separately in
  // AccumulatedDiscountedCostsProto.
  // AccumulatedDiscountedCostsProto EvaluateEachDiscountedAccumulativeCost(
  //     const StatesType& xs, const ControlsType& us, int horizon_clamp,
  //     double gamma) const;

  int problem_costs_size() const { return problem_->costs().size(); }

 protected:
  using FType = typename PROB::FType;
  using DFDxType = typename PROB::DFDxType;
  using DFDuType = typename PROB::DFDuType;
  using DDFDxDxType = typename PROB::DDFDxDxType;
  using DDFDxDuType = typename PROB::DDFDxDuType;
  using DDFDuDxType = typename PROB::DDFDuDxType;
  using DDFDuDuType = typename PROB::DDFDuDuType;
  using GType = typename PROB::GType;
  using DGDxType = typename PROB::DGDxType;
  using DGDuType = typename PROB::DGDuType;
  using DDGDxDxType = typename PROB::DDGDxDxType;
  using DDGDxDuType = typename PROB::DDGDxDuType;
  using DDGDuDxType = typename PROB::DDGDuDxType;
  using DDGDuDuType = typename PROB::DDGDuDuType;
  using ClampInfo = typename PROB::ClampInfo;
  using JType = double;
  using DJDxType = Eigen::Matrix<double, 1, kStateSize>;
  using DDJDxDxType = Eigen::Matrix<double, kStateSize, kStateSize>;

  // Returns the optimal total cost, i.e. Js[0].
  double SolveLinearDdp(const PROB& problem, const StatesType& xs,
                        const ControlsType& us, double curr_cost,
                        StatesType* dxs, ControlsType* dus,
                        ControlsType* dus_open,
                        std::vector<DDGDuDxType>* dus_close_gain, int* k_ne,
                        int* nsd_num) const;

  static bool IsDdJdxdxSemiPositive(const DDJDxDxType& ddJdxdx);
  static bool IsPositiveDefinite(const DDGDuDuType& H);

  template <typename HESSG>
  static HESSG GradJDot(const Eigen::Matrix<double, 1, kStateSize>& grad_j,
                        const std::array<HESSG, kStateSize>& hess_f,
                        bool enable_dynamic_2nd_derivatives) {
    if (enable_dynamic_2nd_derivatives) return HESSG::Zero();
    HESSG hess_g = HESSG::Zero();
    for (int i = 0; i < kStateSize; ++i) {
      hess_g += grad_j[i] * hess_f[i];
    }
    return hess_g;
  }

  double EvaluateCost(const StatesType& xs, const ControlsType& us,
                      std::vector<NamedCostEntry>* named_costs) const;
  double EvaluateCost(const StatesType& xs, const ControlsType& us) const {
    return EvaluateCost(xs, us, /*named_costs=*/nullptr);
  }

  void FitTrajectoryPointToSolverStates(std::vector<TrajectoryPoint> trajectory,
                                        StateType* x0, ControlsType* us,
                                        StatesType* xs) const;

  std::vector<TrajectoryPoint> GenerateTrajectoryPointsFromSolverStates(
      const StatesType& xs, const ControlsType& us, bool s_increasing) const;

  double LineSearchAndEvaluateCost(
      int iteration, const StatesType& tentative_xs,
      const ControlsType& tentative_us, const StatesType& full_dxs,
      const ControlsType& full_dus, double alpha,
      typename MonitorType::OptimizerInspector* oi) const;

  double StepSizeAdjustmentAndEvaluateCost(
      int iteration, const StatesType& xs, const ControlsType& us,
      int k_stepsize, typename MonitorType::OptimizerInspector* oi) const;

  ControlsType OptimizeInitialControl(const StatesType& init_xs,
                                      const ControlsType& init_us);

  DdpOptimizerParamsProto CreateDefaultParams() {
    DdpOptimizerParamsProto params;
    return params;
  }

 private:
  const PROB* problem_;
  int horizon_ = 0;
  std::string owner_;
  int verbosity_ = 0;
  int plan_id_ = 0;
  int not_sd_count_ = 0;
  DdpOptimizerParamsProto params_;
  std::string dopt_tag_;  // Delete after DoptPlanner is deprecated.
  std::unique_ptr<DdpCostManagerMonitor<PROB>> cost_manager_monitor_;

  std::vector<MonitorType*> monitors_;
  std::vector<TrajectoryPoint> init_points_;
};

////////////////////////////////////////////////////////////////////////////////
// Implementations.
template <typename PROB>
DdpOptimizer<PROB>::DdpOptimizer(int plan_id, const PROB* problem, int horizon,
                                 std::string owner, int verbosity,
                                 DdpOptimizerParamsProto params,
                                 std::string dopt_tag)
    : problem_(CHECK_NOTNULL(problem)),
      horizon_(horizon),
      owner_(std::move(owner)),
      verbosity_(verbosity),
      plan_id_(plan_id),
      params_(std::move(params)),
      dopt_tag_(std::move(dopt_tag)) {
  CHECK_GT(horizon_, 0);
  cost_manager_monitor_ =
      std::make_unique<DdpCostManagerMonitor<PROB>>(horizon_);
  for (const auto& helper : problem_->cost_helpers()) {
    cost_manager_monitor_->AddCostHelper(helper.get());
  }
  for (const auto& cost : problem_->costs()) {
    cost_manager_monitor_->AddCost(cost.get());
  }
  AddMonitor(cost_manager_monitor_.get());
}

template <typename PROB>
bool DdpOptimizer<PROB>::IsPositiveDefinite(const DDGDuDuType& H) {
  Eigen::LLT<DDGDuDuType> llt(H);
  if (llt.info() == Eigen::NumericalIssue) {
    return false;
  }
  return true;
}

// According to https://en.wikipedia.org/wiki/Cholesky_decomposition, LDLT
// decomposition: Indefinite matrices have an LDLT decomposition with negative
// entries in D.
template <typename PROB>
bool DdpOptimizer<PROB>::IsDdJdxdxSemiPositive(const DDJDxDxType& ddJdxdx) {
  constexpr double kEps = 1e-9;
  const int size = ddJdxdx.rows();
  StateType D_vec = StateType::Zero();
  DDJDxDxType L = DDJDxDxType::Identity();
  for (int j = 0; j < size; ++j) {
    double D_j = ddJdxdx(j, j);
    for (int k = 0; k < j; ++k) {
      D_j -= Sqr(L(j, k)) * D_vec(k);
    }
    D_vec(j) = D_j;
    if (D_j < -kEps) {
      return false;
    }
    if (std::abs(D_j) < kEps) {
      for (int i = j + 1; i < size; ++i) {
        L(i, j) = 0.0;
      }
    } else {
      double D_j_recip = 1.0 / D_j;
      for (int i = j + 1; i < size; ++i) {
        double L_ij = ddJdxdx(i, j);
        for (int k = 0; k < j; ++k) {
          L_ij -= L(i, k) * L(j, k) * D_vec(k);
        }
        L_ij *= D_j_recip;
        L(i, j) = L_ij;
      }
    }
  }
  return true;
}

template <typename PROB>
double DdpOptimizer<PROB>::SolveLinearDdp(
    const PROB& problem, const StatesType& xs, const ControlsType& us,
    double curr_cost, StatesType* dxs, ControlsType* dus,
    ControlsType* dus_open, std::vector<DDGDuDxType>* dus_close_gain, int* k_ne,
    int* nsd_num) const {
  const StateType x0 = PROB::GetStateAtStep(xs, 0);

  std::vector<JType> Js(horizon_ + 1, {0});
  std::vector<DJDxType> dJdxs(horizon_ + 1);
  std::vector<DDJDxDxType> ddJdxdxs(horizon_ + 1);

  std::vector<ControlType> dus_open_tmp(horizon_);
  std::vector<DDGDuDxType> dus_close_gain_tmp(horizon_);

  // Last step has no J_{k+1}.
  Js[horizon_] = 0.0;
  dJdxs[horizon_] = DJDxType::Zero();
  ddJdxdxs[horizon_] = DDJDxDxType::Zero();

  // Step_size adjustment method Ne
  std::optional<int> k_n_e;
  constexpr double kEps = 1e-8;

  // Backward pass: recursively evaluate J_k.
  // DDPVLOG(3) << "Problem evaluating f and g (and derivatives)";
  std::vector<typename PROB::FDerivatives> fds;
  std::vector<typename PROB::GDerivatives> gds;
  problem.EvaluateFDerivativesForAllSteps(xs, us, &fds);
  problem.AddGDerivativesForAllSteps(xs, us, &gds);
  DCHECK_EQ(fds.size(), horizon_);
  DCHECK_EQ(gds.size(), horizon_);
  for (int k = horizon_ - 1; k >= 0; --k) {
    const FType& f = fds[k].value;
    const DFDxType& dfdx = fds[k].dfdx;
    const DFDuType& dfdu = fds[k].dfdu;
    const DDFDxDxType& ddfdxdx = fds[k].ddfdxdx;
    const DDFDuDxType& ddfdudx = fds[k].ddfdudx;
    const DDFDuDuType& ddfdudu = fds[k].ddfdudu;

    const GType& g = gds[k].value;
    const DGDxType& dgdx = gds[k].dgdx;
    const DGDuType& dgdu = gds[k].dgdu;
    const DDGDxDxType& ddgdxdx = gds[k].ddgdxdx;
    const DDGDuDxType& ddgdudx = gds[k].ddgdudx;
    const DDGDuDuType& ddgdudu = gds[k].ddgdudu;

    const JType& J = Js[k + 1];
    const DJDxType& dJdx = dJdxs[k + 1];
    const DDJDxDxType& ddJdxdx = ddJdxdxs[k + 1];

    const DDGDuDuType A =
        ddgdudu +
        GradJDot(dJdx, ddfdudu, problem.enable_dynamic_2nd_derivatives()) +
        dfdu.transpose() * ddJdxdx * dfdu;
    const DDGDuDxType b_lin =
        ddgdudx +
        GradJDot(dJdx, ddfdudx, problem.enable_dynamic_2nd_derivatives()) +
        dfdu.transpose() * ddJdxdx * dfdx;

    double min_eigen_value = 0.0;
    if (!IsDdJdxdxSemiPositive(ddJdxdx)) {
      Eigen::EigenSolver<DDJDxDxType> es(ddJdxdx);
      min_eigen_value = es.pseudoEigenvalueMatrix().minCoeff();
    }
    const DDGDuDuType A_tilde = A - dfdu.transpose() * min_eigen_value * dfdu;
    if (!IsPositiveDefinite(A_tilde)) {
      ++(*nsd_num);
    }
    const DDGDuDuType Ainv_tilde = A_tilde.inverse();
    const DGDuType b_base_tilde = dgdu + dJdx * dfdu;
    const DDGDuDxType b_lin_tilde =
        b_lin - dfdu.transpose() * min_eigen_value * dfdx;
    const GType c_base_tilde = g + J;

    auto& du_open = dus_open_tmp[k];
    auto& du_close_gain = dus_close_gain_tmp[k];
    du_open = -Ainv_tilde * b_base_tilde.transpose();
    du_close_gain = -Ainv_tilde * b_lin_tilde;
    const auto du_open_transpose = du_open.transpose();
    const auto du_close_gain_transpose = du_close_gain.transpose();

    const double dJ_expected =
        0.5 * du_open_transpose * A * du_open + (b_base_tilde * du_open)(0);
    if (dJ_expected < -kEps && !k_n_e.has_value()) k_n_e = k + 1;

    Js[k] = c_base_tilde + dJ_expected;
    dJdxs[k] = dgdx + dJdx * dfdx + du_open_transpose * A * du_close_gain +
               b_base_tilde * du_close_gain + du_open_transpose * b_lin;
    ddJdxdxs[k] =
        ddgdxdx +
        GradJDot(dJdx, ddfdxdx, problem.enable_dynamic_2nd_derivatives()) +
        dfdx.transpose() * ddJdxdx * dfdx +
        du_close_gain_transpose * A * du_close_gain +
        du_close_gain_transpose * b_lin + b_lin.transpose() * du_close_gain;

    // Symmetrize J hessian. If we don't manually do this, numerical error
    // may accumulate over backward steps and may come to dominate A in the
    // late steps (early time steps in the forward direction), making A^-1
    // significantly inaccurate.
    ddJdxdxs[k] = (ddJdxdxs[k] + ddJdxdxs[k].transpose()) * 0.5;

    // if (UNLIKELY(VLOG_IS_ON(4))) {
    //   DDPVLOG(4) << "---------------------------------- Backward pass step "
    //              << k << " ----------------------------------";

    //   DDPVLOG(4) << "f = " << f.transpose();
    //   DDPVLOG(4) << "dfdx = " << std::endl << dfdx;
    //   DDPVLOG(4) << "dfdu = " << std::endl << dfdu;

    //   DDPVLOG(4) << "g = " << g;
    //   DDPVLOG(4) << "dgdx = " << dgdx;
    //   DDPVLOG(4) << "dgdu = " << dgdu;
    //   DDPVLOG(4) << "ddgdxdx = " << std::endl << ddgdxdx;
    //   DDPVLOG(4) << "ddgdudx = " << std::endl << ddgdudx;
    //   DDPVLOG(4) << "ddgdudu = " << std::endl << ddgdudu;

    //   DDPVLOG(4) << "A = " << std::endl << A_tilde;
    //   DDPVLOG(4) << "Ainv = " << std::endl << Ainv_tilde;
    //   DDPVLOG(4) << "b_base = " << b_base_tilde;
    //   DDPVLOG(4) << "b_lin = " << std::endl << b_lin_tilde;
    //   DDPVLOG(4) << "c = " << c_base_tilde;

    //   DDPVLOG(4) << "J = " << Js[k];
    //   DDPVLOG(4) << "dJdx = " << dJdxs[k];
    //   DDPVLOG(4) << "ddJdxdx = " << std::endl << ddJdxdxs[k];
    // }
  }

  // Forward pass: evaluate u_k^* and x_k^*.
  *dxs = StatesType::Zero(horizon_ * kStateSize);
  *dus = ControlsType::Zero(horizon_ * kControlSize);

  const int k_stepsize_upper = k_n_e.has_value() ? *k_n_e : 0;
  int k_stepsize = -k_stepsize_upper;
  constexpr double kDuLimit = 1e4;
  do {
    k_stepsize = (k_stepsize + k_stepsize_upper) >> 1;
    StateType x = x0;
    // DDPVLOG(4) << "---------------------------------- Step-size adjustment "
    //            << k_stepsize << " ----------------------------------";
    for (int k = 0; k < horizon_; ++k) {
      const StateType dx = x - PROB::GetStateAtStep(xs, k);
      PROB::SetStateAtStep(dx, k, dxs);
      if (k < k_stepsize) {
        x = PROB::GetStateAtStep(xs, k + 1);
        (*dus_close_gain)[k] = DDGDuDxType::Zero();
        PROB::SetControlAtStep(ControlType::Zero(), k, dus);
        PROB::SetControlAtStep(ControlType::Zero(), k, dus_open);
        // if (UNLIKELY(VLOG_IS_ON(4))) {
        //   DDPVLOG(4) << "---------------------------------- Forward pass "
        //                 "adjusted  step "
        //              << k << " ----------------------------------";
        //   DDPVLOG(4) << "x = " << x.transpose();
        //   DDPVLOG(4) << "u = " << (PROB::GetControlAtStep(us,
        //   k)).transpose();
        // }
      } else {
        const ControlType du = dus_open_tmp[k] + dus_close_gain_tmp[k] * dx;
        const ControlType u = PROB::GetControlAtStep(us, k) + du;
        x = problem.EvaluateF(k, x, u);
        (*dus_close_gain)[k] = dus_close_gain_tmp[k];
        PROB::SetControlAtStep(du, k, dus);
        PROB::SetControlAtStep(dus_open_tmp[k], k, dus_open);
        // if (UNLIKELY(VLOG_IS_ON(4))) {
        //   DDPVLOG(4) << "---------------------------------- Forward pass step
        //   "
        //              << k << " ----------------------------------";
        //   DDPVLOG(4) << "dx = " << dx.transpose();
        //   DDPVLOG(4) << "x = " << x.transpose();
        //   DDPVLOG(4) << "du = " << du.transpose();
        //   DDPVLOG(4) << "u = " << u.transpose();
        // }
      }
    }
  } while ((dus->maxCoeff() > kDuLimit || dus->minCoeff() < -kDuLimit) &&
           k_stepsize < (k_stepsize_upper - 1));

  *k_n_e = k_stepsize_upper;
  return Js[0];
}

template <typename PROB>
absl::StatusOr<std::vector<TrajectoryPoint>> DdpOptimizer<PROB>::Solve(
    const std::vector<TrajectoryPoint>& init_trajectory,
    std::vector<std::string>* final_cost_debug_info, double* const final_cost,
    const SolveConfig& config, bool is_compare_weight) const {
  CHECK_NOTNULL(final_cost_debug_info);
  CHECK_NOTNULL(final_cost);
  CHECK_GE(init_trajectory.size(), horizon_);
  constexpr char kTrajectorySmootherOwner[] = "mfob_smoother";
  constexpr char kTrajectoryOptimizerOwner[] = "trajectory_optimizer";
  const std::string& prefix = Log2DDS::TaskPrefix(plan_id_);

  const int xs_length = horizon_ * kStateSize;
  const int us_length = horizon_ * kControlSize;

  *final_cost = std::numeric_limits<double>::max();
  // Fit init trajectory into x0, xs, us.
  StateType x0;
  ControlsType init_us(us_length);
  StatesType init_xs(xs_length);
  FitTrajectoryPointToSolverStates(init_trajectory, &x0, &init_us, &init_xs);
  // DDPVLOG(3) << "init xs = " << init_xs.transpose();
  // DDPVLOG(3) << "init us = " << init_us.transpose();

  // Solver starts.
  // DDPVLOG(1) << "Solve starts";
  typename MonitorType::OptimizerInspector oi;
  StatesType xs = init_xs;
  ControlsType us = init_us;
  for (auto* monitor : monitors_) {
    monitor->OnSolveStart(xs, us, oi);
  }

  double total_cost = 0.0;
  {
    const double init_cost = EvaluateCost(init_xs, init_us, &oi.named_costs);
    // if (VLOG_IS_ON(2)) {
    std::vector<std::string> init_cost_debug;
    DDPVLOG(2) << "Initial cost: " << init_cost;
    const std::string init_cost_debug_first_line =
        is_compare_weight
            ? absl::StrCat(prefix, " Compare weight Initial cost: ", init_cost)
            : absl::StrCat(prefix, " Initial cost: ", init_cost);
    init_cost_debug.push_back(init_cost_debug_first_line);
    for (const auto& [cost_name, cost, is_soft] : oi.named_costs) {
      std::string debug_stream = absl::StrFormat(
          "  [%s]\t cost: %f\t is_soft: %d", cost_name, cost, is_soft);
      DDPVLOG(2) << debug_stream;
      init_cost_debug.push_back(std::move(debug_stream));
    }
    // }
    // Add initial cost debug info to LiReplay.
    if (owner_ == kTrajectorySmootherOwner) {
      Log2DDS::LogDataV2(prefix + "smooth_init_cost_debug",
                         std::move(init_cost_debug));
    }
    if (owner_ == kTrajectoryOptimizerOwner) {
      Log2DDS::LogDataV2(prefix + "init_cost_debug",
                         std::move(init_cost_debug));
    }
    // DDPVLOG(1) << "Initial cost: " << init_cost;
    total_cost = init_cost;
    oi.cost = init_cost;
  }
  // Set line search alphas variables.
  // Slphas vector.
  std::vector<double> line_search_alphas;
  constexpr double kLineSearchAlphaMultiplier = 0.5;
  // Line search must start from alpha = 1.0 to make sure ine search loop will
  // Try full step du update.
  line_search_alphas.push_back(1.0);
  while (line_search_alphas.back() > params_.line_search_min_alpha()) {
    line_search_alphas.push_back(line_search_alphas.back() *
                                 kLineSearchAlphaMultiplier);
  }
  // Start alpha index in alpha vector, set to 1 because full step will be tried
  // when init.
  int alpha_idx = 1;
  const int alpha_count = line_search_alphas.size();
  int line_search_count = 0;

  int iteration;
  const int max_iters = std::min(params_.max_iters(), config.max_iteration);
  std::vector<std::string>& final_cost_debug = *final_cost_debug_info;
  std::vector<std::string> ddp_infos;
  ddp_infos.reserve(max_iters);
  double record_init_cost = 0.0;
  double record_final_cost = 0.0;
  std::string line_type = "";
  int line_search_each_iter = 0;
  for (iteration = 0; iteration < max_iters; ++iteration) {
    DDPVLOG(2) << "Iteration " << iteration << " starts";

    for (auto* monitor : monitors_) {
      monitor->OnIterationStart(iteration, xs, us, oi);
    }

    line_search_each_iter = 1;  // first in backward
    int not_sd_count = 0;
    int k_n_e = horizon_;
    StatesType dxs = StatesType::Zero(xs_length);
    ControlsType dus = ControlsType::Zero(us_length);
    ControlsType dus_open = ControlsType::Zero(us_length);
    std::vector<DDGDuDxType> dus_close_gain(horizon_, DDGDuDxType::Zero());
    const double Js0 =
        SolveLinearDdp(*problem_, xs, us, total_cost, &dxs, &dus, &dus_open,
                       &dus_close_gain, &k_n_e, &not_sd_count);

    if (!problem_->CheckDu(dus, owner_) ||
        !problem_->CheckDu(dus_open, owner_)) {
      return absl::InternalError(owner_ +
                                 " ddp optimizer has unreasonable du.");
    }

    const bool dx_converged =
        dxs.squaredNorm() < Sqr(params_.convergence_tolerance_dx());
    const bool du_converged =
        dus.squaredNorm() < Sqr(params_.convergence_tolerance_du());

    double dcost = 0.0;
    line_type = "init";
    if (!dx_converged && !du_converged) {
      // Line search on du. The control flow below might appear weird, but it is
      // organized so that the calls to RollOutControl() and EvaluateCost() are
      // minimized.
      const double min_acceptable_cost_drop =
          params_.convergence_tolerance_dcost();
      const StatesType line_search_full_dxs = dxs;
      const ControlsType line_search_full_dus = dus;
      const ControlsType line_search_dus_open = dus_open;
      const std::vector<DDGDuDxType> line_search_dus_close_gain =
          dus_close_gain;

      StatesType tentative_xs = xs + dxs;
      ControlsType tentative_us = us + dus;
      const double line_search_init_cost = total_cost;
      // DDPVLOG(4) << "Line search full dxs: "
      //            << line_search_full_dxs.transpose();
      // DDPVLOG(4) << "Line search full dus: "
      //            << line_search_full_dus.transpose();

      // Setting params_.line_search_min_alpha() to 1.0 or higher will disable
      // the line search.
      // Set curr_alpha to first line_search_alphas front.
      double curr_alpha = line_search_alphas.front();
      StatesType curr_xs = tentative_xs;
      ControlsType curr_us = tentative_us;
      double curr_cost = LineSearchAndEvaluateCost(
          iteration, curr_xs, curr_us, line_search_full_dxs,
          line_search_full_dus, curr_alpha, &oi);
      // DDPVLOG(2) << "Line search: init cost: " << line_search_init_cost
      //            << "; Js[0]: " << Js0 << " cost at full step: " << curr_cost
      //            << " du step norm: " << line_search_full_dus.norm();

      if (params_.line_search_to_min()) {
        double next_alpha = curr_alpha;
        StatesType next_xs = curr_xs;
        ControlsType next_us = curr_us;
        double next_cost = curr_cost;
        // Line search is terminated if one of the following conditions is met:
        // 1. Current step cost drop > min_acceptable_cost_drop and next step
        // cost > current step cost.
        // 2. Next step alpha_idx >= alpha_count.
        while (!(curr_cost < line_search_init_cost - min_acceptable_cost_drop &&
                 next_cost > curr_cost) &&
               alpha_idx < alpha_count) {
          curr_alpha = next_alpha;
          curr_xs = next_xs;
          curr_us = next_us;
          curr_cost = next_cost;
          next_alpha = line_search_alphas[alpha_idx];

          const ControlsType processed_dus_open =
              line_search_dus_open * next_alpha;
          StateType x = x0;
          for (int k = 0; k < horizon_; ++k) {
            PROB::SetStateAtStep(x, k, &next_xs);
            const StateType dx = x - PROB::GetStateAtStep(xs, k);
            const ControlType du =
                PROB::GetControlAtStep(processed_dus_open, k) +
                line_search_dus_close_gain[k] * dx;
            const ControlType u = PROB::GetControlAtStep(us, k) + du;
            PROB::SetControlAtStep(u, k, &next_us);
            x = problem_->EvaluateF(k, x, u);
          }

          next_cost = LineSearchAndEvaluateCost(
              iteration, next_xs, next_us, line_search_full_dxs,
              line_search_full_dus, next_alpha, &oi);
          // DDPVLOG(2) << "Line search: next_alpha: " << next_alpha
          //            << " next_cost: " << next_cost
          //            << " curr_alpha: " << curr_alpha
          //            << " curr_cost: " << curr_cost;
          ++alpha_idx;
          ++line_search_count;
        }

        if (next_cost < curr_cost) {
          curr_alpha = next_alpha;
          curr_xs = next_xs;
          curr_us = next_us;
          curr_cost = next_cost;
        }
      } else if (curr_cost > line_search_init_cost - min_acceptable_cost_drop) {
        // Line search is terminated if one of the following conditions is
        // met:
        // 1. Current step cost drop > min_acceptable_cost_drop.
        // 2. Next step alpha_idx >= alpha_count.
        line_type += "+Normal";
        do {
          curr_alpha = line_search_alphas[alpha_idx];
          const ControlsType processed_dus_open =
              line_search_dus_open * curr_alpha;
          StateType x = x0;
          for (int k = 0; k < horizon_; ++k) {
            PROB::SetStateAtStep(x, k, &curr_xs);
            const StateType dx = x - PROB::GetStateAtStep(xs, k);
            const ControlType du =
                PROB::GetControlAtStep(processed_dus_open, k) +
                line_search_dus_close_gain[k] * dx;
            const ControlType u = PROB::GetControlAtStep(us, k) + du;
            PROB::SetControlAtStep(u, k, &curr_us);
            x = problem_->EvaluateF(k, x, u);
          }
          curr_cost = LineSearchAndEvaluateCost(
              iteration, curr_xs, curr_us, line_search_full_dxs,
              line_search_full_dus, curr_alpha, &oi);
          // DDPVLOG(2) << "Line search: curr_alpha: " << curr_alpha
          //            << " curr_cost: " << curr_cost
          //            << " drop: " << line_search_init_cost - curr_cost;
          ++alpha_idx;
          ++line_search_count;
          ++line_search_each_iter;
        } while (
            (curr_cost > line_search_init_cost - min_acceptable_cost_drop) &&
            alpha_idx < alpha_count);
      }

      // Try step size adjustment method if line search failed.
      if (curr_cost > line_search_init_cost - min_acceptable_cost_drop) {
        int k_stepsize = -k_n_e;
        // DDPVLOG(2) << "Step_size adjustment: init_cost:"
        //            << line_search_init_cost
        //            << "cost at full step: " << curr_cost << " k_n_e: " <<
        //            k_n_e;
        line_type += "+Adjustment";
        do {
          curr_xs = xs;
          curr_us = us;
          k_stepsize = (k_stepsize + k_n_e) >> 1;
          StateType x = PROB::GetStateAtStep(curr_xs, k_stepsize);
          for (int k = k_stepsize; k < horizon_; ++k) {
            PROB::SetStateAtStep(x, k, &curr_xs);
            const StateType dx = x - PROB::GetStateAtStep(xs, k);
            const ControlType du =
                PROB::GetControlAtStep(line_search_dus_open, k) +
                line_search_dus_close_gain[k] * dx;
            const ControlType u = PROB::GetControlAtStep(us, k) + du;
            PROB::SetControlAtStep(u, k, &curr_us);
            x = problem_->EvaluateF(k, x, u);
          }

          curr_cost = StepSizeAdjustmentAndEvaluateCost(
              iteration, curr_xs, curr_us, k_stepsize, &oi);
          ++line_search_each_iter;
          // DDPVLOG(2) << "Step_size adjustment: k_stepsize: " << k_stepsize
          //            << " curr_cost: " << curr_cost;
        } while (
            (curr_cost > line_search_init_cost - min_acceptable_cost_drop) &&
            k_stepsize < (k_n_e - 1));
      }

      if (params_.line_search_min_alpha() >= 1.0) {
        // DDPVLOG(2) << "Line search disabled. curr_cost = " << curr_cost
        //            << "; old cost = " << line_search_init_cost;
        dcost = curr_cost - total_cost;
        total_cost = curr_cost;
        tentative_xs = curr_xs;
        tentative_us = curr_us;
        line_type += "+Fail";
      } else if (curr_cost > line_search_init_cost - min_acceptable_cost_drop) {
        // Line search terminated due to alpha limit.
        // DDPVLOG(2) << "Line search failed. This iteration is rejected.";
        line_type += "+Fail";
        tentative_xs = xs;
        tentative_us = us;
        // Final update if not enable line search to min, for print final cost.
        if (!params_.line_search_to_min()) {
          for (auto* monitor : monitors_) {
            monitor->OnLineSearchIterationStart(
                iteration, tentative_xs, tentative_us, line_search_full_dxs,
                line_search_full_dus, curr_alpha, oi);
          }
        }
      } else {
        // Acceptable new cost found. Exit line search.
        // DDPVLOG(2) << "Acceptable cost drop found by line search: alpha = "
        //            << curr_alpha << " cost = " << curr_cost
        //            << " cost drop = " << line_search_init_cost - curr_cost
        //            << " from " << line_search_init_cost;
        line_type += "+Success";
        record_init_cost = total_cost;
        dcost = curr_cost - total_cost;
        total_cost = curr_cost;
        tentative_xs = curr_xs;
        tentative_us = curr_us;
        record_final_cost = curr_cost;

        // Compute next line search start alpha.
        // Line search in next iteration should not start from first alpha, as
        // We think step length may similar to current step length.
        constexpr double kLineSearchDecayFactor = 2.0 / 3.0;
        const int alpha_idx_offset = -1;
        alpha_idx =
            params_.enable_adaptive_alpha()
                ? std::max(1, static_cast<int>(
                                  floor(alpha_idx * kLineSearchDecayFactor) +
                                  alpha_idx_offset))
                : 1;
      }
      // If enable line search to min, need final update to make sure
      // information in cost is coincided to total cost.
      if (params_.line_search_to_min()) {
        for (auto* monitor : monitors_) {
          monitor->OnLineSearchIterationStart(
              iteration, tentative_xs, tentative_us, line_search_full_dxs,
              line_search_full_dus, curr_alpha, oi);
        }
      }

      oi.cost = total_cost;
      dxs = tentative_xs - xs;
      dus = tentative_us - us;
      xs = tentative_xs;
      us = tentative_us;
      DDPVLOG(3) << "End of line search : cost = " << total_cost;
      // End of line search.
    }

    // if (UNLIKELY(VLOG_IS_ON(3))) {
    //   DDPVLOG(3) << "dxs = " << dxs.transpose();
    //   DDPVLOG(3) << "xs = " << xs.transpose();
    //   DDPVLOG(3) << "dus = " << dus.transpose();
    //   DDPVLOG(3) << "us = " << us.transpose();
    // }

    total_cost = EvaluateCost(xs, us, &oi.named_costs);
    oi.cost = total_cost;
    oi.js0 = Js0;
    if (line_type == "Fail") {
      record_init_cost = total_cost;
      record_final_cost = total_cost;
    }
    alpha_idx = std::max(1, std::min(alpha_idx, alpha_count - 1));
    ddp_infos.emplace_back(absl::StrFormat(
        "%s:Iter:%s,Alpaha:%s,LineSearchIter:%s,Init_cost:%s,Final_cost:%s,"
        "LineType:%s,NSD:%s",
        prefix, absl::StrCat(iteration),
        absl::StrCat(line_search_alphas[alpha_idx]),
        absl::StrCat(line_search_each_iter), absl::StrCat(record_init_cost),
        absl::StrCat(record_final_cost), line_type,
        absl::StrCat(not_sd_count)));
    // if (VLOG_IS_ON(2)) {
    final_cost_debug.clear();
    DDPVLOG(2) << "Total cost: " << total_cost;
    const std::string final_cost_debug_first_line =
        is_compare_weight
            ? absl::StrCat(prefix, " Compare weight Total cost: ", total_cost)
            : absl::StrCat(prefix, " Total cost: ", total_cost);
    final_cost_debug.push_back(
        absl::StrCat(prefix, " Total cost: ", total_cost));
    for (const auto& [cost_name, cost, is_soft] : oi.named_costs) {
      std::string debug_stream = absl::StrFormat(
          "  [%s]\t cost: %f\t is_soft: %d", cost_name, cost, is_soft);
      DDPVLOG(2) << debug_stream;
      final_cost_debug.push_back(std::move(debug_stream));
    }
    // }

    for (auto* monitor : monitors_) {
      monitor->OnIterationEnd(iteration, xs, us, oi);
    }

    if (du_converged) {
      // DDPVLOG(1) << "Terminating due to du convergence.";
      break;
    }
    if (dx_converged) {
      // DDPVLOG(1) << "Terminating due to dx convergence.";
      break;
    }
    if (std::abs(dcost) < params_.convergence_tolerance_dcost()) {
      // DDPVLOG(1) << "Terminating due to cost convergence.";
      constexpr double drop_failed_js0_cost = 2.0;
      if ((total_cost - Js0) > drop_failed_js0_cost &&
          owner_ == "trajectory_optimizer") {
      }
      break;
    }
    // End of iteration.
  }
  Log2DDS::LogDataV1(prefix + "ddp_infos", std::move(ddp_infos));

  // Add final cost debug info to LiReplay.
  if (owner_ == kTrajectorySmootherOwner) {
    final_cost_debug.push_back(absl::StrCat(" iteration :", iteration));
    Log2DDS::LogDataV2(prefix + "smooth_final_cost_debug",
                       std::move(final_cost_debug));
  }
  if (owner_ == kTrajectoryOptimizerOwner) {
    final_cost_debug.push_back(absl::StrCat(" iteration :", iteration));
    Log2DDS::LogDataV2(prefix + "final_cost_debug",
                       std::move(final_cost_debug));
  }
  DDPVLOG(2) << "Final cost: " << total_cost;
  // DDPVLOG(3) << "Final xs: " << xs.transpose();
  // DDPVLOG(3) << "Final us: " << us.transpose();

  // Only post process longitudinal control and state.
  // If ddp optimizer drop failed at first iteration, refuse to postprocess
  // trajectory.
  if (iteration > 0 || config.enable_iteration_failure_postprocess) {
    StateType x = x0;
    for (int k = 0; k < horizon_; ++k) {
      PROB::SetStateAtStep(x, k, &xs);
      ControlType u;
      ClampInfo state_clamp_info, control_clamp_info;
      u = problem_->PostProcessLonU(
          PROB::GetControlAtStep(us, k), x,
          k != horizon_ - 1
              ? PROB::GetStateAtStep(xs, k + 1)
              : problem_->EvaluateF(k, x, PROB::GetControlAtStep(us, k)),
          &control_clamp_info, config.forward);
      PROB::SetControlAtStep(u, k, &us);
      const auto x_origin = problem_->EvaluateF(k, x, u);
      x = problem_->PostProcessLonX(x_origin, u, &state_clamp_info);
    }
  }

  *final_cost = oi.cost;
  // Solver end monitor.
  for (auto* monitor : monitors_) {
    monitor->OnSolveEnd(xs, us, oi);
  }

  // Build trajectory points from xs us.
  std::vector<TrajectoryPoint> res =
      GenerateTrajectoryPointsFromSolverStates(xs, us, config.forward);
  if (UNLIKELY(VLOG_IS_ON(3))) {
    for (int k = 0; k < horizon_; ++k) {
      // DDPVLOG(3) << "final_points[" << k << "]: " << res[k].DebugString();
    }
  }

  return res;
}

template <typename PROB>
void DdpOptimizer<PROB>::FitTrajectoryPointToSolverStates(
    std::vector<TrajectoryPoint> trajectory, StateType* x0, ControlsType* us,
    StatesType* xs) const {
  CHECK_NOTNULL(x0);
  CHECK_NOTNULL(us);
  CHECK_NOTNULL(xs);

  // Preprocess trajectory.
  {
    CHECK_GE(trajectory.size(), horizon_) << "Invalid trajectory size.";
    for (int k = 0; k < horizon_; ++k) {
      trajectory[k].set_t(k * problem_->dt());
    }
    // TODO: Make sure each consecutive points' theta diff
    // aren't jumping 2*pi.
    trajectory.front().set_theta(
        trajectory[1].theta() +
        NormalizeAngle(trajectory.front().theta() - trajectory[1].theta()));
  }

  // Generate initial controls and states.
  // We do not rollout initial states but use fitted initial states.
  *x0 = problem_->FitInitialState(trajectory);
  *us = problem_->FitControl(trajectory, *x0);
  *xs = problem_->FitState(trajectory);
}

template <typename PROB>
std::vector<TrajectoryPoint>
DdpOptimizer<PROB>::GenerateTrajectoryPointsFromSolverStates(
    const StatesType& xs, const ControlsType& us, bool s_increasing) const {
  // Build trajectory points from xs us.
  std::vector<TrajectoryPoint> res;
  res.resize(horizon_);
  for (int k = 0; k < horizon_; ++k) {
    TrajectoryPoint& point = res[k];
    problem_->ExtractTrajectoryPoint(k, PROB::GetStateAtStep(xs, k),
                                     PROB::GetControlAtStep(us, k), &point);
  }

  // Re-fill s of all trajectory points.
  if (problem_->enable_post_process()) {
    res.front().set_s(0.0);
    for (int i = 1; i < horizon_; ++i) {
      const double d = (res[i].pos() - res[i - 1].pos()).norm();
      if (s_increasing) {
        res[i].set_s(res[i - 1].s() + d);
      } else {
        res[i].set_s(res[i - 1].s() - d);
      }
    }
  }

  return res;
}

template <typename PROB>
double DdpOptimizer<PROB>::EvaluateCost(
    const StatesType& xs, const ControlsType& us,
    std::vector<NamedCostEntry>* named_costs) const {
  std::vector<double> costs(problem_->costs().size(), 0.0);

  if (named_costs != nullptr) named_costs->clear();

  for (int i = 0; i < problem_->costs().size(); ++i) {
    const auto divided_g =
        problem_->costs()[i]->SumGForAllSteps(xs, us, horizon_);
    costs[i] += divided_g.sum();
    if (named_costs != nullptr) {
      for (const auto& cost : divided_g.gs()) {
        named_costs->push_back(cost);
      }
    }
  }
  const double total_cost = std::accumulate(costs.begin(), costs.end(), 0.0);
  // if (UNLIKELY(VLOG_IS_ON(4))) {
  //   DDPVLOG(4) << "Total cost: " << total_cost;
  //   for (const auto& [cost_name, cost, is_soft] : *named_costs) {
  //     DDPVLOG(4) << "  [" << cost_name << "] cost: " << cost
  //                << ", is_soft: " << is_soft;
  //   }
  // }
  return total_cost;
}

template <typename PROB>
double DdpOptimizer<PROB>::LineSearchAndEvaluateCost(
    int iteration, const StatesType& tentative_xs,
    const ControlsType& tentative_us, const StatesType& full_dxs,
    const ControlsType& full_dus, double alpha,
    typename MonitorType::OptimizerInspector* oi) const {
  // DDPVLOG(4) << "Evaluating cost for line search iteration " << iteration
  //            << " with alpha " << alpha;
  for (auto* monitor : monitors_) {
    monitor->OnLineSearchIterationStart(iteration, tentative_xs, tentative_us,
                                        full_dxs, full_dus, alpha, *oi);
  }
  const double cost = EvaluateCost(tentative_xs, tentative_us);
  oi->cost = cost;
  for (auto* monitor : monitors_) {
    monitor->OnLineSearchIterationEnd(iteration, tentative_xs, tentative_us,
                                      full_dxs, full_dus, alpha, cost, *oi);
  }
  return cost;
}

template <typename PROB>
double DdpOptimizer<PROB>::StepSizeAdjustmentAndEvaluateCost(
    int iteration, const StatesType& xs, const ControlsType& us, int k_stepsize,
    typename MonitorType::OptimizerInspector* oi) const {
  // DDPVLOG(4) << "Evaluating cost for step-size adjustment iteration "
  //            << iteration << " with step " << k_stepsize;
  for (auto* monitor : monitors_) {
    monitor->OnStepSizeAdjustmentIterationStart(iteration, xs, us, k_stepsize,
                                                *oi);
  }
  const double cost = EvaluateCost(xs, us);
  oi->cost = cost;
  for (auto* monitor : monitors_) {
    monitor->OnStepSizeAdjustmentIterationEnd(iteration, xs, us, k_stepsize,
                                              cost, *oi);
  }
  return cost;
}

// template <typename PROB>
// AccumulatedDiscountedCostsProto
// DdpOptimizer<PROB>::EvaluateEachDiscountedAccumulativeCost(
//     const StatesType& xs, const ControlsType& us, int horizon_clamp,
//     double gamma) const {
//   horizon_clamp = std::min(horizon_clamp, horizon_);
//   const int soft_feature_idx = 0;
//   const int hard_feature_idx = 1;
//   const std::string object_feature_name = "ObjectCost";
//   const std::string curb_msd_feature_name = "MsdStaticBoundaryCostV2_curb";
//   const std::string sold_line_msd_feature_name =
//       "SolidLineMsdStaticBoundaryCost";
//   double solid_line_msd_feature_cost_value = 0.0;
//   std::vector<std::pair<std::string, double>> curb_msd_cost;
//   curb_msd_cost.resize(/*object_cost_buffet_size=*/2, {"", 0.0});
//   std::vector<std::pair<std::string, double>> object_cost;
//   object_cost.resize(/*object_cost_buffet_size=*/2, {"", 0.0});

//   // AccumulatedDiscountedCostsProto costs_proto;

//   for (int i = 0; i < problem_->costs().size(); ++i) {
//     double gamma_k = 1.0;
//     std::vector<NamedCostEntry> divide_cost;
//     for (int k = 0; k < horizon_clamp; ++k) {
//       const auto cost_k = problem_->costs()[i]->EvaluateGWithDebugInfo(
//           k, PROB::GetStateAtStep(xs, k), PROB::GetControlAtStep(us, k),
//           /*using_scale=*/false);
//       const auto& gs = cost_k.gs();
//       const int gs_size = gs.size();
//       if (divide_cost.size() < gs_size) {
//         const int old_size = divide_cost.size();
//         divide_cost.resize(gs_size);
//         for (int idx = old_size; idx < gs_size; ++idx) {
//           divide_cost[idx].name = gs[idx].name;
//           divide_cost[idx].is_soft = gs[idx].is_soft;
//           divide_cost[idx].value = 0.0;
//         }
//       }
//       for (int idx = 0; idx < gs_size; ++idx) {
//         divide_cost[idx].value += gamma_k * gs[idx].value;
//       }
//       gamma_k *= gamma;
//     }

//     const auto& cost_type = problem_->costs()[i]->cost_type();
//     if (cost_type == Cost<PROB>::CostType::UNKNOWN) {
//       // LOG_ERROR << problem_->costs()[i]->name()
//       //             << " is an unknown type cost.";
//     }
//     if (cost_type == Cost<PROB>::CostType::GROUP_OBJECT) {
//       for (int idx = 0; idx < divide_cost.size(); ++idx) {
//         if (divide_cost[idx].is_soft) {
//           object_cost[soft_feature_idx].second += divide_cost[idx].value;
//         } else {
//           object_cost[hard_feature_idx].second += divide_cost[idx].value;
//         }
//       }
//       continue;
//     }
//     // BANDIT: SolidLineMsdStaticBoundaryCost doesn't exist all the
//     // time, so need to fake it with 0.0 value if it doesn't exist.
//     if (cost_type == Cost<PROB>::CostType::SOLID_LINE_MSD_STATIC_BOUNDARY) {
//       CHECK_EQ(divide_cost.size(), 1);
//       CHECK_EQ(divide_cost[0].name, sold_line_msd_feature_name);

//       solid_line_msd_feature_cost_value = divide_cost[0].value;
//       continue;
//     }

//     // BANDIT: CURB_MSD_STATIC_BOUNDARY_V2 and
//     // UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2 share the same cost weight so
//     // they need to be merged together by their corresponding gain.
//     if (cost_type == Cost<PROB>::CostType::CURB_MSD_STATIC_BOUNDARY_V2) {
//       for (int idx = 0; idx < divide_cost.size(); ++idx) {
//         if (divide_cost[idx].is_soft) {
//           curb_msd_cost[soft_feature_idx].second +=
//               kCurbGain * divide_cost[idx].value;
//         } else {
//           curb_msd_cost[hard_feature_idx].second +=
//               kCurbGain * divide_cost[idx].value;
//         }
//       }
//       continue;
//     }
//     if (cost_type ==
//         Cost<PROB>::CostType::UTURN_RIGHT_CURB_MSD_STATIC_BOUNDARY_V2) {
//       for (int idx = 0; idx < divide_cost.size(); ++idx) {
//         if (divide_cost[idx].is_soft) {
//           curb_msd_cost[soft_feature_idx].second +=
//               kUTurnCurbGain * divide_cost[idx].value;
//         } else {
//           curb_msd_cost[hard_feature_idx].second +=
//               kUTurnCurbGain * divide_cost[idx].value;
//         }
//       }
//       continue;
//     }

//     for (int idx = 0; idx < divide_cost.size(); ++idx) {
//       FeatureCostProto* feature_cost = costs_proto.add_feature_costs();
//       feature_cost->set_feature(divide_cost[idx].name);
//       feature_cost->set_cost(divide_cost[idx].value);
//     }
//   }
//   for (int idx = 0; idx < object_cost.size(); ++idx) {
//     FeatureCostProto* object_feature_cost = costs_proto.add_feature_costs();
//     if (idx == soft_feature_idx) {
//       object_feature_cost->set_feature(SoftNameString + object_feature_name);
//     } else if (idx == hard_feature_idx) {
//       object_feature_cost->set_feature(HardNameString + object_feature_name);
//     }
//     object_feature_cost->set_cost(object_cost[idx].second);
//   }
//   // Add Soft and Hard CurbMsdStaticBoundaryCost at the end to preserve the
//   // order of cost.
//   for (int idx = 0; idx < curb_msd_cost.size(); ++idx) {
//     FeatureCostProto* curb_msd_feature_cost =
//     costs_proto.add_feature_costs(); if (idx == soft_feature_idx) {
//       curb_msd_feature_cost->set_feature(SoftNameString +
//                                          curb_msd_feature_name);
//     } else if (idx == hard_feature_idx) {
//       curb_msd_feature_cost->set_feature(HardNameString +
//                                          curb_msd_feature_name);
//     }
//     curb_msd_feature_cost->set_cost(curb_msd_cost[idx].second);
//   }
//   // Add SolidLineMsdStaticBoundaryCost at the end to preserve the order of
//   // cost.
//   FeatureCostProto* solid_line_msd_feature_cost =
//       costs_proto.add_feature_costs();
//   solid_line_msd_feature_cost->set_feature(sold_line_msd_feature_name);
//   solid_line_msd_feature_cost->set_cost(solid_line_msd_feature_cost_value);

//   // if (UNLIKELY(VLOG_IS_ON(2))) {
//   //   DDPVLOG(2) << "Discounted Accumulative Cost: ";
//   //   for (int i = 0; i < costs_proto.feature_costs_size(); ++i) {
//   //     DDPVLOG(2) << "  [" << costs_proto.feature_costs(i).feature()
//   //                << "] cost: " << costs_proto.feature_costs(i).cost();
//   //   }
//   // }
//   return costs_proto;
// }

#undef DDPVLOG
}  // namespace planning
}  // namespace st

#endif  // ONBOARD_PLANNER_OPTIMIZATION_DDP_DDP_OPTIMIZER_H_
