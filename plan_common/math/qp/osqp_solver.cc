

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "plan_common/math/qp/osqp_solver.h"

// IWYU pragma: no_include <ostream>
#include "Eigen/SparseCore"
#include "absl/cleanup/cleanup.h"
#include "absl/strings/str_cat.h"
#include "plan_common/math/qp/osqp_return_status.h"

namespace st {

OsqpSolver::OsqpSolver(
    const SMatXd& A,                  // NOLINT(readability-identifier-naming)
    const VecXd& b, const SMatXd& G,  // NOLINT(readability-identifier-naming)
    const VecXd& g,
    // NOLINTNEXTLINE(readability-identifier-naming)
    const SMatXd& H, const VecXd& h)
    : SparseQpSolver(A.triangularView<Eigen::Upper>(), b, G, g, H, h) {}

absl::Status OsqpSolver::Solve() {
  auto settings = std::make_unique<OSQPSettings>();
  osqp_set_default_settings(settings.get());
  settings->alpha = 1.0;
  settings->eps_abs = 1e-7;
  settings->eps_rel = 1e-7;
  settings->verbose = false;

  // According to https://github.com/google/osqp-cpp#faq
  // To make the simulation result deterministic, set the `adaptive_rho` as
  // false, "but this could significantly slow down OSQP's convergence."
  settings->adaptive_rho = false;

  return Solve(*settings);
}

absl::Status OsqpSolver::Solve(const OSQPSettings& settings) {
  // instead of long long. Then this copying for A can be avoided (because
  // SMatXd::StorageIndex is int).
  A_.uncompress();
  std::vector<c_float> A_values;       // NOLINT(readability-identifier-naming)
  std::vector<c_int> A_inner_indices;  // NOLINT(readability-identifier-naming)
  std::vector<c_int> A_outer_starts;   // NOLINT(readability-identifier-naming)
  A_values.reserve(A().nonZeros());
  A_inner_indices.reserve(A().nonZeros());
  A_outer_starts.reserve(nx() + 1);
  A_outer_starts.push_back(0);
  for (int i = 0; i < nx(); ++i) {
    for (int j = 0; j < A().innerNonZeroPtr()[i]; ++j) {
      A_values.push_back(A().valuePtr()[A().outerIndexPtr()[i] + j]);
      A_inner_indices.push_back(
          A().innerIndexPtr()[A().outerIndexPtr()[i] + j]);
    }
    A_outer_starts.push_back(A_values.size());
  }
  CHECK_EQ(A_values.size(), A().nonZeros());

  G_.uncompress();
  H_.uncompress();
  std::vector<c_float> constraint_values;
  std::vector<c_int> constraint_inner_indices;
  std::vector<c_int> constraint_outer_starts;
  constraint_values.reserve(G().nonZeros() + H().nonZeros());
  constraint_inner_indices.reserve(G().nonZeros() + H().nonZeros());
  constraint_outer_starts.reserve(nx() + 1);
  constraint_outer_starts.push_back(0);
  for (int i = 0; i < nx(); ++i) {
    for (int j = 0; j < G().innerNonZeroPtr()[i]; ++j) {
      constraint_values.push_back(G().valuePtr()[G().outerIndexPtr()[i] + j]);
      constraint_inner_indices.push_back(
          G().innerIndexPtr()[G().outerIndexPtr()[i] + j]);
    }
    for (int j = 0; j < H().innerNonZeroPtr()[i]; ++j) {
      constraint_values.push_back(H().valuePtr()[H().outerIndexPtr()[i] + j]);
      constraint_inner_indices.push_back(
          H().innerIndexPtr()[H().outerIndexPtr()[i] + j] + ng());
    }
    constraint_outer_starts.push_back(constraint_values.size());
  }
  CHECK_EQ(constraint_values.size(), G().nonZeros() + H().nonZeros());

  constexpr double kConstraintUpperBound = 1e10;  // Essentially unbounded.
  VecXd l = VecXd::Zero(ng() + nh());
  VecXd u = VecXd::Zero(ng() + nh());
  l.segment(0, ng()) = g();
  u.segment(0, ng()) = g();
  l.segment(ng(), nh()) = h();
  u.segment(ng(), nh()) = VecXd::Constant(nh(), kConstraintUpperBound);

  auto data = std::make_unique<OSQPData>();
  data->n = nx();
  data->m = ng() + nh();
  data->P = csc_matrix(data->n, data->n, A_values.size(), A_values.data(),
                       A_inner_indices.data(), A_outer_starts.data());
  data->q = const_cast<double*>(b().data());
  data->A = csc_matrix(
      data->m, data->n, constraint_values.size(), constraint_values.data(),
      constraint_inner_indices.data(), constraint_outer_starts.data());
  data->l = const_cast<double*>(l.data());
  data->u = const_cast<double*>(u.data());

  OSQPWorkspace* work = nullptr;
  const int exit_flag =
      osqp_setup(&work, data.get(), const_cast<OSQPSettings*>(&settings));
  if (exit_flag != 0) {
    return absl::InternalError(
        absl::StrCat("Fail to setup OSQP, exit code: ", exit_flag));
  }

  const int result = osqp_solve(work);

  x_ = VecXd::Zero(nx());
  for (int i = 0; i < nx(); ++i) {
    x_[i] = work->solution->x[i];
  }

  const absl::Cleanup cleaner = [&] {
    osqp_cleanup(work);
    c_free(data->A);
    c_free(data->P);
  };

  const auto status_val = work->info->status_val;
  const auto status_info = OsqpReturnStatusToString(status_val);
  const auto solver_info = OsqpReturnSolverErrorsToString(result);
  if (result != 0 || status_val != OSQP_SOLVED) {
    const auto error_msg = absl::StrCat(
        "Fail to solve problem with OSQP, solver value: ", solver_info,
        " status: ", status_info, " iterations: ", work->info->iter,
        " time: ", work->info->run_time, "s", " x[0]: ", x_[0]);
    return absl::InternalError(error_msg);
  }

  return absl::OkStatus();
}

}  // namespace st
