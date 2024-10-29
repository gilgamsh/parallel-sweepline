#include "sweepline.hpp"

namespace drc {

void sweepline_top_check(database &db) { sweepline_check(db.top_gpu_segs); }

void sweepline_bot_check(database &db) { sweepline_check(db.bot_gpu_segs); };

} // namespace drc