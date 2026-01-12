

#ifndef ONBOARD_PLANNER_MATH_QP_OSQP_RETURN_STATUS_H_
#define ONBOARD_PLANNER_MATH_QP_OSQP_RETURN_STATUS_H_

#include <string>

namespace st {

/*
  Status values information, reference resource:
  https://osqp.org/docs/interfaces/status_values.html
*/
inline std::string OsqpReturnStatusToString(int status) {
  switch (status) {
    case 1:
      return "OSQP_SOLVED";
    case 2:
      return "OSQP_SOLVED_INACCURATE";
    case -2:
      return "OSQP_MAX_ITER_REACHED";
    case -3:
      return "OSQP_PRIMAL_INFEASIBLE";
    case 3:
      return "OSQP_PRIMAL_INFEASIBLE_INACCURATE";
    case -4:
      return "OSQP_DUAL_INFEASIBLE";
    case 4:
      return "OSQP_DUAL_INFEASIBLE_INACCURATE";
    case -5:
      return "OSQP_SIGINT";
    case -6:
      return "OSQP_TIME_LIMIT_REACHED";
    case -10:
      return "OSQP_UNSOLVED";
    case -7:
      return "OSQP_NON_CVX";
    default:
      return "Unknown_status_val";
  }
}

// Solver errors information, reference resource is same as above.
inline std::string OsqpReturnSolverErrorsToString(int value) {
  switch (value) {
    case 0:
      return "OSQP_SOLVER_FINISHED";
    case 1:
      return "OSQP_DATA_VALIDATION_ERROR";
    case 2:
      return "OSQP_SETTINGS_VALIDATION_ERROR";
    case 3:
      return "OSQP_LINSYS_SOLVER_LOAD_ERROR";
    case 4:
      return "OSQP_LINSYS_SOLVER_INIT_ERROR";
    case 5:
      return "OSQP_NONCVX_ERROR";
    case 6:
      return "OSQP_MEM_ALLOC_ERROR";
    case 7:
      return "OSQP_WORKSPACE_NOT_INIT";
    default:
      return "Unknown_solver_val";
  }
}

}  // namespace st

#endif  // ONBOARD_PLANNER_MATH_QP_OSQP_RETURN_STATUS_H_
