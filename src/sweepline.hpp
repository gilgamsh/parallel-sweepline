#pragma once
#ifndef DRC_SWEEPLINE
#define DRC_SWEEPLINE

#include "drc_database.hpp"


namespace drc {


void sweepline_check(std::vector<gpu_seg_net_radius> &segs);

void sweepline_top_check(database &db);

void sweepline_bot_check(database &db);


}


#endif