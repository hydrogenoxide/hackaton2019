#pragma once

#include "xodr/xodr_map.h"

using namespace aid::xodr;

/**
 * @brief Find out if this lane is visible on our mash.
 */
bool lane_is_visible(LaneSection::Lane* lane);

/**
 * @brief Gets the visible lanes of given lane section.
 */
std::vector<LaneSection::Lane> visible_lanes(LaneSection* laneSection);

/**
 * @brief Get the number of visible lanes on the left of the reference line.
 */
int num_visible_lanes_left(LaneSection* laneSection);

/**
 * @brief Get the number of visible lanes on the right of the reference line.
 */
int num_visible_lanes_right(LaneSection* laneSection);

/**
 * @brief Get the number of visible lanes.
 */
int num_visible_lanes(LaneSection* laneSection);