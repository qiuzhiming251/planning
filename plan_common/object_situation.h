//
// Created by xxx on 8/2/21.
//

#ifndef COMMON_OBJECT_SITUATION
#define COMMON_OBJECT_SITUATION
#include <deque>

#include "plan_common/type_def.h"
namespace ad_byd::planning {

class ObjectMotion {
 public:
  ObjectMotion(/* args */) = default;
  ~ObjectMotion() = default;
  // longitudinal speed
  double lon_speed() const { return lon_speed_; }
  void set_lon_speed(const double &val) { lon_speed_ = val; }

  // longitudinal acceleration
  double lon_acc() const { return lon_acc_; }
  void set_lon_acc(const double &val) { lon_acc_ = val; }

  // lateral speed
  double lat_speed() const { return lat_speed_; }
  void set_lat_speed(const double &val) { lat_speed_ = val; }

  // response time for brake
  double response_time() const { return response_time_; }
  void set_response_time(const double &val) { response_time_ = val; }

  // proper brake for yield front car
  double proper_brake() const { return proper_brake_; }
  void set_proper_brake(const double &val) { proper_brake_ = val; }

  // distance to stop line of the junction
  double distance_to_junction() const { return distance_to_junction_; }
  void set_distance_junction(const double &val) { distance_to_junction_ = val; }

 private:
  double lon_speed_ = 0.0;      // m/s
  double lon_acc_ = 0.0;        // m/s2
  double lat_speed_ = 0.0;      // m/s
  double response_time_ = 1.0;  // s
  double proper_brake_ = Constants::PROPER_BRAKE;
  double distance_to_junction_ = 0.0;  // m
};

class ObjectSituation {
 public:
  ObjectSituation() = default;
  ~ObjectSituation() = default;

  /// get longitudinal sighed distance
  /// @return positive if in front,negative if behind,zero if overlap
  double lon_distance() const { return lon_distance_; }
  void set_lon_distance(const double &val) { lon_distance_ = val; }

  double lon_max() const { return lon_max_; }
  void set_lon_max(const double &val) { lon_max_ = val; }

  double lon_min() const { return lon_min_; }
  void set_lon_min(const double &val) { lon_min_ = val; }

  /// get lateral sighed distance
  /// @return positive if at left,negative if right,zero if overlap
  double lat_distance() const { return lat_distance_; }
  void set_lat_distance(const double &val) { lat_distance_ = val; }

  // motion info include v, a, response time
  const ObjectMotion &motion() const { return motion_; }
  ObjectMotion &mutableMotion() { return motion_; }

  void Reset();

 private:
  double lon_distance_ = 0.0;  // m
  double lat_distance_ = 0.0;  // m
  double lon_max_ = 0.0;       // m
  double lon_min_ = 0.0;       // m
  ObjectMotion motion_;
};
}  // namespace ad_byd::planning

#endif  // COMMON_OBJECT_SITUATION
