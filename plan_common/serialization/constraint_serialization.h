#ifndef _PLAN_COMMON_SERIALIZATION_CONSTRAINT_SERIALIZATION_H_
#define _PLAN_COMMON_SERIALIZATION_CONSTRAINT_SERIALIZATION_H_

#include <cereal/cereal.hpp>
#include "modules/cnoa_pnc/planning/proto/constraint.pb.h"

namespace st::planning {
template <typename Archive>
void serialize(Archive& ar,
               ConstraintProto::LeadingObjectProto& leading_object_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    leading_object_proto.ParseFromString(serialized_proto);
  } else {
    leading_object_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               ConstraintProto::SpeedRegionProto& speed_region_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    speed_region_proto.ParseFromString(serialized_proto);
  } else {
    speed_region_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, ConstraintProto::StopLineProto& stop_line_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    stop_line_proto.ParseFromString(serialized_proto);
  } else {
    stop_line_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               ConstraintProto::PathStopLineProto& path_stop_line_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    path_stop_line_proto.ParseFromString(serialized_proto);
  } else {
    path_stop_line_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               ConstraintProto::PathSpeedRegionProto& path_speed_region_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    path_speed_region_proto.ParseFromString(serialized_proto);
  } else {
    path_speed_region_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar, ConstraintProto::AvoidLineProto& avoid_line_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    avoid_line_proto.ParseFromString(serialized_proto);
  } else {
    avoid_line_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}

template <typename Archive>
void serialize(Archive& ar,
               ConstraintProto::SpeedProfileProto& speed_profile_proto) {
  std::string serialized_proto;
  if (Archive::is_loading::value) {
    ar(CEREAL_NVP(serialized_proto));
    speed_profile_proto.ParseFromString(serialized_proto);
  } else {
    speed_profile_proto.SerializeToString(&serialized_proto);
    ar(CEREAL_NVP(serialized_proto));
  }
}
}  // namespace st::planning

#endif