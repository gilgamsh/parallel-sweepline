#pragma once
#ifndef DRC_DATABASE
#define DRC_DATABASE

#include "../dsnDataBase.h"

namespace drc {

/*
for both cpu and cuda
geometry: point, segment, polygon,
signal: geometry+ netinfo, layerinfo
*/

/*
use the point in point.h point_2d
*/

struct segment {
  point_2d pt0, pt1;
};

struct signal_segment {
  int net_id;
  segment seg;
};

// maybe use this way to represent the metal
struct metal_segment {
  int net_id;
  double radius;
  segment seg;
};

// maybe use this Mincowsky sum
struct wiring_wire {
  int net_id;
  int layer_id;
  double width;
  segment seg;
};

struct wiring_via {
  int net_id;
  int layer_id;
  double width;
  point_2d pt;
};

struct metal_polygon {
  int net_id;
  int layer_id;
  points_2d pts;
};


template <typename T>
struct seg_net_radius{
  T x0,y0,x1,y1;
  int net_id;
};

using gpu_seg_net_radius = seg_net_radius<int>;

struct database {

  double clearance;
  double largest_radius=0.1;
  point_2d min_xy=point_2d{1e6,1e6},max_xy={0,0};

  std::vector<metal_polygon> metal_polygons;
  std::vector<wiring_wire> wiring_wires;
  std::vector<wiring_via> wiring_vias;

  std::vector<metal_polygon> top_metal_polygons;
  std::vector<wiring_wire> top_wiring_wires;
  std::vector<wiring_via> top_wiring_vias;

  std::vector<metal_polygon> bot_metal_polygons;
  std::vector<wiring_wire> bot_wiring_wires;
  std::vector<wiring_via> bot_wiring_vias;

  // expand with clearance , not used
  std::vector<signal_segment> top_signal_segments;
  std::vector<signal_segment> bot_signal_segments;

  // have width
  std::vector<metal_segment> top_metal_segments;
  std::vector<metal_segment> bot_metal_segments;

  // gpu data 
  std::vector<gpu_seg_net_radius> top_gpu_segs;
  std::vector<gpu_seg_net_radius> bot_gpu_segs;

  void init(dsnDataBase &db); // init the database from the dsnDataBase
  void init_info(dsnDataBase &db); // init the info for the database

  void init_gpu_data(); // init the gpu data from the database
  void modify_metal_segments(); // change the metal segments to two types
  void make2_2();
  void make4_2();
  void make4_4();

  void print_metal_polygons();
  void print_wiring_wires();
  void print_wiring_vias();

  void print_top_metal_polygons();
  void print_top_wiring_wires();
  void print_top_wiring_vias();

  void print_bot_metal_polygons();
  void print_bot_wiring_wires();
  void print_bot_wiring_vias();
};

} // namespace drc
#endif