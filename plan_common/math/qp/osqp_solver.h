

#ifndef ST_PLANNING_MATH_QP_OSQP_SOLVER
#define ST_PLANNING_MATH_QP_OSQP_SOLVER

#include "osqp/osqp.h"
//
#include "absl/status/status.h"
#include "plan_common/math/eigen.h"
#include "plan_common/math/qp/sparse_qp_solver.h"

namespace st {

// Wrapper for the OSQP sparse solver.
class OsqpSolver : public SparseQpSolver {
 public:
  // A can be either symmetric or upper triangular, but only the upper
  // triangular part of A will be used.
  OsqpSolver(const SMatXd& A, const VecXd& b, const SMatXd& G, const VecXd& g,
             const SMatXd& H, const VecXd& h);
  virtual ~OsqpSolver() = default;

  absl::Status Solve() override;
  absl::Status Solve(const OSQPSettings& settings);
};

}  // namespace st

#endif  // ST_PLANNING_MATH_QP_OSQP_SOLVER
