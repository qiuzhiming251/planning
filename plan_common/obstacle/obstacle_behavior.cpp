

#include "plan_common/obstacle/obstacle_behavior.h"
#include "plan_common/math/double.h"

namespace ad_byd {
namespace planning {
namespace obstacle {
using Vec2d = ::ad_byd::planning::math::Vec2d;

void PredictionBehavior::SetId(const int32_t& id) {
  id_ = id;
  has_id_ = true;
}

void PredictionBehavior::SetIsValid(const bool& is_valid) {
  is_valid_ = is_valid;
  has_is_valid_ = true;
}

void PredictionBehavior::SetProbability(const double& probability) {
  probability_ = probability;
  has_probability_ = true;
}

void PredictionBehavior::SetTimeSpan(const double& time_span) {
  time_span_ = time_span;
  has_time_span_ = true;
}

void PredictionBehavior::PushRuleOrder(const int32_t& rule_order) {
  has_rule_order_ = true;
  rule_order_.emplace_back(rule_order);
}

void PredictionBehavior::SetModelTrajectory(
    std::vector<ModelPredictionPoint>&& model_trajectory) {
  model_trajectory_ = std::move(model_trajectory);
  has_model_trajectory_ = true;
}

void PredictionBehavior::SetPredictionConstrains(
    std::vector<PredictionConstrain>&& constriains) {
  prediction_constrains_ = std::move(constriains);
  has_prediction_constrains_ = true;
}

void PredictionBehavior::SetPredictorType(const PredictorType& predictor_type) {
  has_predictor_type_ = true;
  predictor_type_ = predictor_type;
}

void ObstacleBehavior::SetPredictionBehaviors(
    std::vector<PredictionBehavior>&& prediction_behaviors) {
  prediction_behaviors_ = std::move(prediction_behaviors);
  has_prediction_behaviors_ = true;
}

void ObstacleBehavior::SetIntentionPolyline(
    std::vector<IntentionPolyline>&& intention_polylines) {
  intention_polylines_ = std::move(intention_polylines);
  has_intion_polylines_ = true;
}
}  // namespace obstacle
}  // namespace planning
}  // namespace ad_byd