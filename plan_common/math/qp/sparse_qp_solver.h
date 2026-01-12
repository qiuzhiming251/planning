

#ifndef ONBOARD_MATH_QP_SPARSE_QP_SOLVER_H_
#define ONBOARD_MATH_QP_SPARSE_QP_SOLVER_H_

#include <string>

#include "plan_common/log.h"

#include "absl/status/status.h"
#include "plan_common/math/eigen.h"

namespace st {

class SparseQpSolver {
 public:
  // Quadratic programming notation:
  // Minimize \frac12 x^T A x + b^T x over x, subject to Gx = g and Hx >= h.
  //
  // Number of states: nx = x.rows()
  // A is nx by nx (cost)
  // b is nx by 1
  // G is ng by nx (ng equality constraints)
  // g is ng by 1
  // H is nh by nx (nh inequality constraints)
  // h is nh by 1
  SparseQpSolver(const SMatXd& A, const VecXd& b, const SMatXd& G,
                 const VecXd& g, const SMatXd& H, const VecXd& h)
      : A_(A), b_(b), G_(G), g_(g), H_(H), h_(h) {
    CHECK_EQ(A_.rows(), A_.cols());
    // CHECK_EQ(A_, A_.transpose());
    CHECK_EQ(A_.rows(), G_.cols());
    CHECK_EQ(A_.rows(), H_.cols());
    CHECK_EQ(G_.rows(), g_.rows());
    CHECK_EQ(H_.rows(), h_.rows());
  }

  virtual ~SparseQpSolver() {}

  const SMatXd& A() const { return A_; }
  const VecXd& b() const { return b_; }
  const SMatXd& G() const { return G_; }
  const VecXd& g() const { return g_; }
  const SMatXd& H() const { return H_; }
  const VecXd& h() const { return h_; }

  const VecXd& x() const { return x_; }

  int nx() const { return A_.rows(); }
  int ng() const { return G_.rows(); }
  int nh() const { return H_.rows(); }

  virtual absl::Status Solve() = 0;

  std::string DebugString() const {
    std::stringstream ss;
    ss << "A: \n"
       << A_ << "\n"
       << "b: " << b_.transpose() << "\n"
       << "G: \n"
       << G_ << "\n"
       << "g: " << g_.transpose() << "\n"
       << "H: \n"
       << H_ << "\n"
       << "h: " << h_.transpose() << std::endl;
    return ss.str();
  }

 protected:
  SMatXd A_;
  VecXd b_;
  SMatXd G_;
  VecXd g_;
  SMatXd H_;
  VecXd h_;

  VecXd x_;
};

}  // namespace st

#endif  // ONBOARD_MATH_QP_SPARSE_QP_SOLVER_H_
