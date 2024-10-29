#include "rtree.hpp"
#include "omp.h"
namespace drc {

void check(
    std::vector<segment_net_radius> &results, segment_net_radius it,
    std::vector<std::pair<segment_net_radius, segment_net_radius>> &outputs,
    database &db) {
  for (auto &res : results) {
    auto net_id = res.second.first;
    auto radius = res.second.second;
    auto seg = res.first;

    auto dis = bg::distance(seg, it.first);
    if (net_id == it.second.first and net_id != -1 and it.second.first != -1) {
      continue;
    }
    if (dis < radius + it.second.second) {
      outputs.push_back({res, it});
    }
  }
}

void rtree_top_check(database &db) {
  std::vector<std::pair<segment_net_radius, segment_net_radius>> top_outputs;

  std::vector<segment_net_radius> top_segment_nets;

  for (auto &metal_segment : db.top_metal_segments) {
    DSegment seg(DPoint(metal_segment.seg.pt0.x(), metal_segment.seg.pt0.y()),
                 DPoint(metal_segment.seg.pt1.x(), metal_segment.seg.pt1.y()));
    top_segment_nets.push_back(
        {seg, {metal_segment.net_id, metal_segment.radius}});
  }
  std::cout << "top_segment_nets.size() " << top_segment_nets.size()
            << std::endl;

  RTree top_rtree(top_segment_nets.begin(), top_segment_nets.end());

  for (auto &seg_net_radius : top_segment_nets) {
    auto enlarge = db.largest_radius + seg_net_radius.second.second;
    auto seg = seg_net_radius.first;
    auto xl = std::min(seg.first.x(), seg.second.x()) - enlarge;
    auto xh = std::max(seg.first.x(), seg.second.x()) + enlarge;
    auto yl = std::min(seg.first.y(), seg.second.y()) - enlarge;
    auto yh = std::max(seg.first.y(), seg.second.y()) + enlarge;
    DBox query_box(DPoint(xl, yl), DPoint(xh, yh));
    std::vector<segment_net_radius> result;
    top_rtree.query(bgi::intersects(query_box), std::back_inserter(result));
    check(result, seg_net_radius, top_outputs, db);
  }

  std::cout << "top_outputs.size() " << top_outputs.size() << std::endl;
  if constexpr (print_top_results) {
    for (auto &it : top_outputs) {
      std::cout << "top_outputs net:" << it.first.second.first << " "
                << it.second.second.first << std::endl;
      std::cout << "top_outputs seg0:" << it.first.first.first.x() << " "
                << it.first.first.first.y() << " " << it.first.first.second.x()
                << " " << it.first.first.second.y() << std::endl;
      std::cout << "top_outputs seg1:" << it.second.first.first.x() << " "
                << it.second.first.first.y() << " "
                << it.second.first.second.x() << " "
                << it.second.first.second.y() << std::endl;
      std::cout << "top_outputs radius:" << it.first.second.second << " "
                << it.second.second.second << std::endl;
    }
  }
}

void rtree_bot_check(database &db) {
  std::vector<std::pair<segment_net_radius, segment_net_radius>> bot_outputs;

  std::vector<segment_net_radius> bot_segment_nets;

  for (auto &metal_segment : db.bot_metal_segments) {
    DSegment seg(DPoint(metal_segment.seg.pt0.x(), metal_segment.seg.pt0.y()),
                 DPoint(metal_segment.seg.pt1.x(), metal_segment.seg.pt1.y()));
    bot_segment_nets.push_back(
        {seg, {metal_segment.net_id, metal_segment.radius}});
  }
  std::cout << "bot_segment_nets.size() " << bot_segment_nets.size()
            << std::endl;

  RTree bot_rtree(bot_segment_nets.begin(), bot_segment_nets.end());

  for (auto &seg_net_radius : bot_segment_nets) {
    auto enlarge = db.largest_radius + seg_net_radius.second.second;
    auto net_id = seg_net_radius.second.first;
    auto seg = seg_net_radius.first;
    auto xl = std::min(seg.first.x(), seg.second.x()) - enlarge;
    auto xh = std::max(seg.first.x(), seg.second.x()) + enlarge;
    auto yl = std::min(seg.first.y(), seg.second.y()) - enlarge;
    auto yh = std::max(seg.first.y(), seg.second.y()) + enlarge;
    DBox query_box(DPoint(xl, yl), DPoint(xh, yh));
    std::vector<segment_net_radius> result;
    bot_rtree.query(bgi::intersects(query_box), std::back_inserter(result));
    check(result, seg_net_radius, bot_outputs, db);
  }
  std::cout << "bot_outputs.size() " << bot_outputs.size() << std::endl;

  if constexpr (print_bot_results) {
    for (auto &it : bot_outputs) {
      std::cout << "bot_outputs net:" << it.first.second.first << " "
                << it.second.second.first << std::endl;
      std::cout << "bot_outputs seg0:" << it.first.first.first.x() << " "
                << it.first.first.first.y() << " " << it.first.first.second.x()
                << " " << it.first.first.second.y() << std::endl;
      std::cout << "bot_outputs seg1:" << it.second.first.first.x() << " "
                << it.second.first.first.y() << " "
                << it.second.first.second.x() << " "
                << it.second.first.second.y() << std::endl;
      std::cout << "bot_outputs radius:" << it.first.second.second << " "
                << it.second.second.second << std::endl;
    }
  }
}

void rtree_check_mt(std::vector<metal_segment> &metal_segments, database &db) {
  std::vector<std::pair<segment_net_radius, segment_net_radius>> outputs;
  std::vector<segment_net_radius> segment_nets;

  for (int i = 0; i < metal_segments.size(); i++) {
    auto &metal_segment = metal_segments[i];
    DSegment seg(DPoint(metal_segment.seg.pt0.x(), metal_segment.seg.pt0.y()),
                 DPoint(metal_segment.seg.pt1.x(), metal_segment.seg.pt1.y()));
    segment_nets.push_back({seg, {metal_segment.net_id, metal_segment.radius}});
  }

  RTree rtree(segment_nets.begin(), segment_nets.end());

  #pragma omp parallel for num_threads(8)
  for (int i = 0; i < segment_nets.size(); i++) {
    auto &seg_net_radius = segment_nets[i];
    auto enlarge =  seg_net_radius.second.second;
    auto &seg = seg_net_radius.first;
    auto xl = std::min(seg.first.x(), seg.second.x()) - enlarge;
    auto xh = std::max(seg.first.x(), seg.second.x()) + enlarge;
    auto yl = std::min(seg.first.y(), seg.second.y()) - enlarge;
    auto yh = std::max(seg.first.y(), seg.second.y()) + enlarge;
    DBox query_box(DPoint(xl, yl), DPoint(xh, yh));
    std::vector<segment_net_radius> result;
    rtree.query(bgi::intersects(query_box), std::back_inserter(result));

    check(result, seg_net_radius, outputs,db);
  }
}

void rtree_top_check_mt(database &db){rtree_check_mt(db.top_metal_segments,db);}
void rtree_bot_check_mt(database &db){rtree_check_mt(db.bot_metal_segments,db);}

} // namespace drc
