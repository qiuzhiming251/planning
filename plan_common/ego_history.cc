
#include "ego_history.h"
namespace st::planning {

EgoHistory::EgoHistory() {}

EgoHistory::~EgoHistory() {}

void EgoHistory::CleanExceeded() {
  if (frames_.size() >= MAX_EGO_FRAME_NUM) {
    frames_.pop_front();
  }
}

void EgoHistory::AddNewFrame(EgoFrame ego_new_frame) {
  frames_.push_back(ego_new_frame);
}

void EgoHistory::UpdateEgoHistory(EgoFrame ego_curr_frame) {
  this->CleanExceeded();
  this->AddNewFrame(ego_curr_frame);
}
}  // namespace st::planning