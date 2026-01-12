

#ifndef ONBOARD_MAPS_MAP_OR_DIE_MACROS_H_
#define ONBOARD_MAPS_MAP_OR_DIE_MACROS_H_
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "plan_common/base/macros.h"
//#include "lite/logging.h"
//#include "lite/qissue_trans.h"

#include "plan_common/maps/semantic_map_defs.h"
#include "plan_common/maps/smm_proto_util.h"
//#include "q_issue.pb.h"
//#include "plan_common/util/source_location.h"

// TODO delete
// ============== PROTO related macros ==============
#define SMM_LANE_PROTO_OR_RETURN(assign_val, smm, id, return_val)  \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindLaneProto, smm, \
                      st::mapping::ElementId(id),                  \
                      SMM_CONCAT(find_id, __LINE__), false, return return_val)
#define SMM_SECTION_PROTO_OR_RETURN(assign_val, smm, id, return_val)  \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindSectionProto, smm, \
                      st::mapping::SectionId(id),                     \
                      SMM_CONCAT(find_id, __LINE__), false, return return_val)

#define SMM_LANE_PROTO_OR_BREAK(assign_val, smm, id)               \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindLaneProto, smm, \
                      st::mapping::ElementId(id),                  \
                      SMM_CONCAT(find_id, __LINE__), false, break)
#define SMM_SECTION_PROTO_OR_BREAK(assign_val, smm, id)               \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindSectionProto, smm, \
                      st::mapping::SectionId(id),                     \
                      SMM_CONCAT(find_id, __LINE__), false, break)
#define SMM_LANE_BOUNDARY_PROTO_OR_BREAK(assign_val, smm, id)              \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindLaneBoundaryProto, smm, \
                      st::mapping::ElementId(id),                          \
                      SMM_CONCAT(find_id, __LINE__), false, break)

#define SMM_LANE_PROTO_OR_CONTINUE(assign_val, smm, id)            \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindLaneProto, smm, \
                      st::mapping::ElementId(id),                  \
                      SMM_CONCAT(find_id, __LINE__), false, continue)
#define SMM_SECTION_PROTO_OR_CONTINUE(assign_val, smm, id)            \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindSectionProto, smm, \
                      st::mapping::SectionId(id),                     \
                      SMM_CONCAT(find_id, __LINE__), false, continue)
#define SMM_LANEBOUNDARY_PROTO_OR_CONTINUE(assign_val, smm, id)            \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindLaneBoundaryProto, smm, \
                      st::mapping::ElementId(id),                          \
                      SMM_CONCAT(find_id, __LINE__), false, continue)

#define SMM_LANE_PROTO_OR_ERROR(assign_val, smm, id)               \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindLaneProto, smm, \
                      st::mapping::ElementId(id),                  \
                      SMM_CONCAT(find_id, __LINE__), false,        \
                      return absl::NotFoundError(                  \
                          absl::StrCat("Cannot find element id: ", id)))

#define SMM_SECTION_PROTO_OR_ERROR(assign_val, smm, id)               \
  SMM_PROTO_OR_HANDLE(assign_val, st::mapping::FindSectionProto, smm, \
                      st::mapping::SectionId(id),                     \
                      SMM_CONCAT(find_id, __LINE__), false,           \
                      return absl::NotFoundError(                     \
                          absl::StrCat("Cannot find section id: ", id)))

// ============== SWALLOW ERROR DEFULT ==============
#define SMM_ASSIGN_SECTION_OR_BREAK(assign_val, smm, id)            \
  SMM_ASSIGN_OR_BREAK(assign_val, smm, st::mapping::FindSectionPtr, \
                      st::mapping::SectionId(id), false)
#define SMM2_ASSIGN_SECTION_OR_BREAK(assign_val, v2smm, id)           \
  SMM_ASSIGN_OR_BREAK(assign_val, v2smm, st::mapping::FindSectionPtr, \
                      st::mapping::SectionId(id), false)

#define SMM_ASSIGN_LANE_OR_BREAK(assign_val, smm, id)            \
  SMM_ASSIGN_OR_BREAK(assign_val, smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), false)
#define SMM2_ASSIGN_LANE_OR_BREAK(assign_val, v2smm, id)           \
  SMM_ASSIGN_OR_BREAK(assign_val, v2smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), false)

#define SMM_ASSIGN_SECTION_OR_ERROR(assign_val, smm, id)            \
  SMM_ASSIGN_OR_ERROR(assign_val, smm, st::mapping::FindSectionPtr, \
                      st::mapping::SectionId(id), false)
#define SMM_ASSIGN_LANE_OR_ERROR(assign_val, smm, id)            \
  SMM_ASSIGN_OR_ERROR(assign_val, smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), false)
#define SMM2_ASSIGN_LANE_OR_ERROR(assign_val, smm, id)           \
  SMM_ASSIGN_OR_ERROR(assign_val, smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), false)

#define SMM_ASSIGN_SECTION_OR_RETURN(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindSectionPtr,  \
                       st::mapping::SectionId(id), false, return_val)
#define SMM2_ASSIGN_SECTION_OR_RETURN(assign_val, v2smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, v2smm, st::mapping::FindSectionPtr,   \
                       st::mapping::SectionId(id), false, return_val)

#define SMM_ASSIGN_LANE_OR_RETURN(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindLanePtr,  \
                       st::mapping::ElementId(id), false, return_val)
#define SMM2_ASSIGN_LANE_OR_RETURN(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindLanePtr,   \
                       st::mapping::ElementId(id), false, return_val)

#define SMM_ASSIGN_LANE_OR_CONTINUE(assign_val, smm, id)            \
  SMM_ASSIGN_OR_CONTINUE(assign_val, smm, st::mapping::FindLanePtr, \
                         st::mapping::ElementId(id), false)

#define SMM2_ASSIGN_LANE_OR_CONTINUE(assign_val, v2smm, id)           \
  SMM_ASSIGN_OR_CONTINUE(assign_val, v2smm, st::mapping::FindLanePtr, \
                         st::mapping::ElementId(id), false)

#define SMM_ASSIGN_SECTION_OR_CONTINUE(assign_val, smm, id)            \
  SMM_ASSIGN_OR_CONTINUE(assign_val, smm, st::mapping::FindSectionPtr, \
                         st::mapping::SectionId(id), false)
#define SMM2_ASSIGN_SECTION_OR_CONTINUE(assign_val, v2smm, id)           \
  SMM_ASSIGN_OR_CONTINUE(assign_val, v2smm, st::mapping::FindSectionPtr, \
                         st::mapping::SectionId(id), false)

// ============== ISSUE KICKOUT ==============
#define SMM_ASSIGN_SECTION_OR_BREAK_ISSUE(assign_val, smm, id)      \
  SMM_ASSIGN_OR_BREAK(assign_val, smm, st::mapping::FindSectionPtr, \
                      st::mapping::SectionId(id), true)
#define SMM2_ASSIGN_SECTION_OR_BREAK_ISSUE(assign_val, v2smm, id)     \
  SMM_ASSIGN_OR_BREAK(assign_val, v2smm, st::mapping::FindSectionPtr, \
                      st::mapping::SectionId(id), true)

#define SMM_ASSIGN_LANE_OR_BREAK_ISSUE(assign_val, smm, id)      \
  SMM_ASSIGN_OR_BREAK(assign_val, smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), true)
#define SMM2_ASSIGN_LANE_OR_BREAK_ISSUE(assign_val, v2smm, id)     \
  SMM_ASSIGN_OR_BREAK(assign_val, v2smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), true)

#define SMM_ASSIGN_SECTION_OR_ERROR_ISSUE(assign_val, smm, id)      \
  SMM_ASSIGN_OR_ERROR(assign_val, smm, st::mapping::FindSectionPtr, \
                      st::mapping::SectionId(id), true)
#define SMM_ASSIGN_LANE_OR_ERROR_ISSUE(assign_val, smm, id) \
  SMM_ASSIGN_OR_ERROR(assign_val, smm, st::mapping::FindLanePtr, id, true)
#define SMM2_ASSIGN_LANE_OR_ERROR_ISSUE(assign_val, smm, id)     \
  SMM_ASSIGN_OR_ERROR(assign_val, smm, st::mapping::FindLanePtr, \
                      st::mapping::ElementId(id), true)

#define SMM_ASSIGN_SECTION_OR_RETURN_ISSUE(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindSectionPtr,        \
                       st::mapping::SectionId(id), true, return_val)
#define SMM2_ASSIGN_SECTION_OR_RETURN_ISSUE(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindSectionPtr,         \
                       st::mapping::SectionId(id), true, return_val)

#define SMM_ASSIGN_LANE_OR_RETURN_ISSUE(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindLanePtr,        \
                       st::mapping::ElementId(id), true, return_val)
#define SMM2_ASSIGN_LANE_OR_RETURN_ISSUE(assign_val, smm, id, return_val) \
  SMM_ASSIGN_OR_RETURN(assign_val, smm, st::mapping::FindLanePtr,         \
                       st::mapping::ElementId(id), true, return_val)

#define SMM_ASSIGN_LANE_OR_CONTINUE_ISSUE(assign_val, smm, id)      \
  SMM_ASSIGN_OR_CONTINUE(assign_val, smm, st::mapping::FindLanePtr, \
                         st::mapping::ElementId(id), true)
#define SMM2_ASSIGN_LANE_OR_CONTINUE_ISSUE(assign_val, v2smm, id)     \
  SMM_ASSIGN_OR_CONTINUE(assign_val, v2smm, st::mapping::FindLanePtr, \
                         st::mapping::ElementId(id), true)

#define SMM_ASSIGN_SECTION_OR_CONTINUE_ISSUE(assign_val, smm, id)      \
  SMM_ASSIGN_OR_CONTINUE(assign_val, smm, st::mapping::FindSectionPtr, \
                         st::mapping::SectionId(id), true)
#define SMM2_ASSIGN_SECTION_OR_CONTINUE_ISSUE(assign_val, v2smm, id)     \
  SMM_ASSIGN_OR_CONTINUE(assign_val, v2smm, st::mapping::FindSectionPtr, \
                         st::mapping::SectionId(id), true)

// ============== INNER MACROS ==============
#define SMM_ASSIGN_OR_ERROR(assign_val, smm, name, id, kickout) \
  SMM_ASSIGN_OR_HANDLE(                                         \
      assign_val, smm, name, id, SMM_CONCAT(find_id, __LINE__), \
      SMM_CONCAT(find_ptr, __LINE__), kickout,                  \
      return absl::NotFoundError(absl::StrCat("Cannot find id:", id)))

#define SMM_ASSIGN_OR_RETURN(assign_val, smm, name, id, kickout, return_val) \
  SMM_ASSIGN_OR_HANDLE(                                                      \
      assign_val, smm, name, id, SMM_CONCAT(find_id, __LINE__),              \
      SMM_CONCAT(find_ptr, __LINE__), kickout, return return_val)

#define SMM_ASSIGN_OR_CONTINUE(assign_val, smm, name, id, kickout) \
  SMM_ASSIGN_OR_HANDLE(assign_val, smm, name, id,                  \
                       SMM_CONCAT(find_id, __LINE__),              \
                       SMM_CONCAT(find_ptr, __LINE__), kickout, continue)

#define SMM_ASSIGN_OR_CONTINUE(assign_val, smm, name, id, kickout) \
  SMM_ASSIGN_OR_HANDLE(assign_val, smm, name, id,                  \
                       SMM_CONCAT(find_id, __LINE__),              \
                       SMM_CONCAT(find_ptr, __LINE__), kickout, continue)

#define SMM_ASSIGN_OR_BREAK(assign_val, smm, name, id, kickout) \
  SMM_ASSIGN_OR_HANDLE(assign_val, smm, name, id,               \
                       SMM_CONCAT(find_id, __LINE__),           \
                       SMM_CONCAT(find_ptr, __LINE__), kickout, break)

#define SMM_PROTO_OR_HANDLE(assign_val, func, smm, id, map_to_find_id, \
                            kickout, ...)                              \
  const auto map_to_find_id = (id);                                    \
  auto assign_val = func(smm, map_to_find_id);                         \
  if (UNLIKELY(assign_val == nullptr)) {                               \
    auto error_message =                                               \
        absl::StrCat("Cannot find map element: ", map_to_find_id);     \
    __VA_ARGS__;                                                       \
  }

#define SMM_ASSIGN_OR_HANDLE(assign_val, smm, func, id, map_to_find_id, ptr, \
                             kickout, ...)                                   \
  const auto map_to_find_id = (id);                                          \
  const auto ptr = func(smm, map_to_find_id);                                \
  if (UNLIKELY(ptr == nullptr)) {                                            \
    auto error_message =                                                     \
        absl::StrCat("Cannot find map element: ", map_to_find_id);           \
    __VA_ARGS__;                                                             \
  }                                                                          \
  const auto& assign_val = *ptr

#define SMM_CONCAT3(x, y, z) SMM_CONCAT(x, SMM_CONCAT(y, z))
#define SMM_CONCAT(x, y) SMM_CONCAT_IMPL(x, y)
#define SMM_CONCAT_IMPL(x, y) x##y

#endif  // ONBOARD_MAPS_MAP_OR_DIE_MACROS_H
