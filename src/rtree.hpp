#pragma once
#ifndef DRC_RTREE
#define DRC_RTREE

#include "../pcbBoost.h"
#include "drc_database.hpp"

namespace drc {

using segment_net_radius = std::pair<DSegment, std::pair<int, double>>;
using RTree = bgi::rtree<segment_net_radius, bgi::quadratic<16>>;

// extern 
// extern std::vector<segment_net_radius> bot_outputs;

void rtree_top_check(database &db);
void rtree_bot_check(database &db);

void rtree_check_mt(std::vector<metal_segment> &metal_segments,database &db);
void rtree_top_check_mt(database &db);
void rtree_bot_check_mt(database &db);


} // namespace drc

#endif