#include "visible_lanes.h"

bool lane_is_visible(LaneSection::Lane *lane) {
  return lane->type() != LaneType::NONE;
}

std::vector<LaneSection::Lane> visible_lanes(LaneSection *laneSection) {
  std::vector<LaneSection::Lane> visible_lanes;
  std::vector<LaneSection::Lane> lanes = laneSection->lanes();

  int i = 0;
  for (LaneSection::Lane lane : lanes) {
    if (lane_is_visible(&lane)) {
      visible_lanes.push_back(lane);
    } else {
      if (i != 0 && i != lanes.size() - 1) {
	std::cerr
	    << "A lane that was neither first nor last was invisible! (i= " << i
	    << ", all=" << lanes.size() - 1 << ") Check for possible bugs!"
	    << std::endl;
      }
    }
    i++;
  }

  return visible_lanes;
}

int num_visible_lanes_left(LaneSection *laneSection) {
  std::vector<LaneSection::Lane> lanes = visible_lanes(laneSection);
  int counter = 0;
  for (LaneSection::Lane lane : lanes) {
    if (static_cast<int>(lane.id()) > 0) {
      counter++;
    }
  }
  return counter;
}

int num_visible_lanes_right(LaneSection *laneSection) {
  std::vector<LaneSection::Lane> lanes = visible_lanes(laneSection);
  int counter = 0;
  for (LaneSection::Lane lane : lanes) {
    if (static_cast<int>(lane.id()) < 0) {
      counter++;
    }
  }
  return counter;
}

int num_visible_lanes(LaneSection *laneSection) {
  return num_visible_lanes_left(laneSection) +
         num_visible_lanes_right(laneSection);
}
