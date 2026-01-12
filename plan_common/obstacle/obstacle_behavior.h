

#ifndef AD_BYD_PLANNING_COMMON_OBSTACLE_OBSTACLE_BEHAVIOR_H
#define AD_BYD_PLANNING_COMMON_OBSTACLE_OBSTACLE_BEHAVIOR_H

#include <vector>

#include "plan_common/obstacle/prediction_constrain.h"
#include "plan_common/type_def.h"
#include "plan_common/maps/lane.h"
#include "plan_common/math/vec2d.h"

namespace ad_byd {
namespace planning {
namespace obstacle {
using Vec2d = math::Vec2d;
using LaneConstPtr = std::shared_ptr<const ::ad_byd::planning::Lane>;
struct ModelPredictionPoint {
  Vec2d position;
  // TODO: sigma ....
  double t = 0.0;
  Vec2d velocity;
  Vec2d acc;
  double heading = 0.0;
};

struct IntentionPolyline {
  double score = 0.0;
  std::string id = "";
  std::vector<LaneConstPtr> lanes;
};

class PredictionBehavior {
 public:
  PredictionBehavior() = default;
  ~PredictionBehavior() = default;

  void SetId(const int32_t& id);
  void SetIsValid(const bool& is_valid);
  void SetProbability(const double& probability);
  void SetTimeSpan(const double& time_span);
  void PushRuleOrder(const int32_t& rule_order);
  void SetModelTrajectory(std::vector<ModelPredictionPoint>&& model_trajectory);
  void SetPredictionConstrains(std::vector<PredictionConstrain>&& constriains);
  void SetPredictorType(const PredictorType& predictor_type);

  const int32_t& id() const { return id_; }
  const bool& IsValid() const { return is_valid_; }
  const double& probability() const { return probability_; }
  const double& time_span() const { return time_span_; }
  const std::vector<int32_t>& rule_order() const { return rule_order_; }
  const std::vector<ModelPredictionPoint>& model_trajectory() const {
    return model_trajectory_;
  }
  std::vector<ModelPredictionPoint>* mutable_model_trajectory() {
    return &model_trajectory_;
  }
  const std::vector<PredictionConstrain>& prediction_constrains() const {
    return prediction_constrains_;
  }
  std::vector<PredictionConstrain>* mutable_prediction_constrains() {
    return &prediction_constrains_;
  }
  const PredictorType& predictor_type() const { return predictor_type_; }

  bool HasId() const { return has_id_; }
  bool HasIsValid() const { return has_is_valid_; }
  bool HasProbability() const { return has_probability_; }
  bool HasTimeSpan() const { return has_time_span_; }
  bool HasRuleOrder() const { return has_rule_order_; }
  bool HasModelTrajectory() const { return has_model_trajectory_; }
  bool HasPredictionConstrains() const { return has_prediction_constrains_; }
  bool HasPredictorType() const { return has_predictor_type_; }

 private:
  int32_t id_ = 0;
  bool is_valid_ = false;
  double probability_ = 0.0;
  double time_span_ = 0.0;
  std::vector<int32_t> rule_order_;
  std::vector<ModelPredictionPoint> model_trajectory_;
  std::vector<PredictionConstrain> prediction_constrains_;
  PredictorType predictor_type_ = PredictorType::UNKNOWN;

  bool has_id_ = false;
  bool has_is_valid_ = false;
  bool has_probability_ = false;
  bool has_time_span_ = false;
  bool has_rule_order_ = false;
  bool has_model_trajectory_ = false;
  bool has_prediction_constrains_ = false;
  bool has_predictor_type_ = false;
};

class ObstacleBehavior {
 public:
  ObstacleBehavior() = default;
  ~ObstacleBehavior() = default;

  void SetIntentionPolyline(
      std::vector<IntentionPolyline>&& intention_polylines);
  void SetPredictionBehaviors(
      std::vector<PredictionBehavior>&& prediction_behaviors);

  const std::vector<PredictionBehavior>& prediction_behaviors() const {
    return prediction_behaviors_;
  }
  const std::vector<IntentionPolyline>& intention_polylines() const {
    return intention_polylines_;
  }
  std::vector<PredictionBehavior>* mutable_prediction_behaviors() {
    return &prediction_behaviors_;
  }
  std::vector<IntentionPolyline>* mutable_intention_polylines() {
    return &intention_polylines_;
  }

  bool HasPredictionBehaviors() const { return has_prediction_behaviors_; }

 private:
  std::vector<PredictionBehavior> prediction_behaviors_;
  std::vector<IntentionPolyline> intention_polylines_;
  bool has_prediction_behaviors_ = false;
  bool has_intion_polylines_ = false;
};

}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd
#endif  // AD_BYD_PLANNING_COMMON_OBSTACLE_OBSTACLE_BEHAVIOR_H