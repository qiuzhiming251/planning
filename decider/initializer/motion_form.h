

#ifndef ONBOARD_PLANNER_INITIALIZER_MOTION_FORM_H_
#define ONBOARD_PLANNER_INITIALIZER_MOTION_FORM_H_

#include <memory>
#include <utility>
#include <vector>

#include "decider/initializer/geometry/geometry_form.h"
#include "decider/initializer/geometry/geometry_state.h"
#include "decider/initializer/motion_state.h"

namespace st::planning {

enum MotionFormType {
  CONST_ACCEL_MOTION = 1,
  STATIONARY_MOTION = 2,
  COMPLETE_MOTION = 3,
};
struct SampledMotionFormStates {
  std::vector<MotionState> const_interval_states;
  std::vector<MotionState> equal_interval_states;
};
// This represents a short trajectory.
class MotionForm {
 public:
  // The time duration of the motion.
  virtual double duration() const = 0;
  virtual MotionState GetStartMotionState() const = 0;
  virtual MotionState GetEndMotionState() const = 0;
  virtual MotionState State(double t) const = 0;
  virtual const GeometryForm* geometry() const = 0;
  virtual MotionFormType type() const = 0;
  // Discretized by the required motion sample step
  virtual SampledMotionFormStates SampleStates() const = 0;
  // Equal interval states without l derivatives calculation.
  virtual std::vector<MotionState> SampleEqualIntervalStates() const = 0;
  virtual ~MotionForm() {}
  // The trajectory sample step for const time interval sampling. When it is 2,
  // it means sampling every two trajectory time interval (which is 0.1
  // normally, meaning in intializer we sample every 0.2s).
  static constexpr int kConstTimeIntervalSampleStep = 3;
  // The trajectory sample step configuration for equal time interval sampling.
  // A motion form is sampled with equal time interval.
  static constexpr int kMinEqualTimeIntervalSampleStep = 4;
  static constexpr int kMaxEqualTimeIntervalSampleStep = 11;
  static constexpr double kDesireEqualTimeInterval = 0.5;
};

class ConstAccelMotion final : public MotionForm {
 public:
  // Create by v and a
  explicit ConstAccelMotion(double init_v, double init_a,
                            const GeometryForm* geometry);
  // Create by v0 and v1. Use pair to differentiate constructors' signatures.
  explicit ConstAccelMotion(std::pair<double, double> v_pair,
                            const GeometryForm* geometry);

  double duration() const override { return duration_; }
  MotionState GetStartMotionState() const override;
  MotionState GetEndMotionState() const override;

  MotionState State(double t) const override;

  const GeometryForm* geometry() const override { return geometry_; }
  MotionFormType type() const override {
    return MotionFormType::CONST_ACCEL_MOTION;
  }

  SampledMotionFormStates SampleStates() const override;
  std::vector<MotionState> SampleEqualIntervalStates() const override;

 private:
  double init_v_ = 0.0;
  double a_ = 0.0;
  double duration_ = 0.0;
  // Time of which the motion stops before the end of geometry. Set to max
  // if sdc still moving at the end node.
  double stop_time_ = 0.0;
  double stop_distance_ = 0.0;
  // Not owned.
  const GeometryForm* geometry_ = nullptr;
};

class StationaryMotion final : public MotionForm {
 public:
  explicit StationaryMotion(double duration, const StationaryGeometry* geometry)
      : duration_(duration), geometry_(geometry) {}

  explicit StationaryMotion(double duration, GeometryState state)
      : duration_(duration),
        geometry_form_(std::make_unique<StationaryGeometry>(std::move(state))),
        geometry_(geometry_form_.get()) {}
  double duration() const override { return duration_; }
  MotionState GetStartMotionState() const override;
  MotionState GetEndMotionState() const override;
  MotionState State(double t) const override;

  const GeometryForm* geometry() const override { return geometry_; }
  MotionFormType type() const override {
    return MotionFormType::STATIONARY_MOTION;
  }
  SampledMotionFormStates SampleStates() const override;
  std::vector<MotionState> SampleEqualIntervalStates() const override;

 private:
  std::vector<MotionState> Sample(double d_t) const;
  double duration_ = 0.0;
  // Optional: maybe null.
  std::unique_ptr<StationaryGeometry> geometry_form_;

  // Maybe not owned.
  const GeometryForm* geometry_ = nullptr;
};

}  // namespace st::planning
#endif  // ONBOARD_PLANNER_INITIALIZER_MOTION_FORM_H_
