#include "drc_database.hpp"

namespace drc {

void database::init(dsnDataBase &db) {

  if constexpr (print_netlist) {
    std::cout << "netlist: " << std::endl;
    for (auto &net : db.getNets()) {
      std::cout << "net: " << net.getName() << " netid: " << net.getId()
                << std::endl;
    }
  }

  clearance = db.getLargestClearance();

  int max_net_id = -1;

  // geometry from wires and vias
  for (auto &net : db.getNets()) {
    int net_id = net.getId();
    max_net_id = std::max(max_net_id, net_id);
    for (auto &wire : net.getSegments()) {
      wiring_wire t_wiring_wire;
      t_wiring_wire.net_id = net_id;
      t_wiring_wire.layer_id = db.getLayerId(wire.getLayer());
      t_wiring_wire.width = wire.getWidth();
      t_wiring_wire.seg = segment{wire.getPos().front(), wire.getPos().back()};
      wiring_wires.push_back(t_wiring_wire);
    }
    for (auto &via : net.getVias()) {
      wiring_via t_wiring_via;
      t_wiring_via.net_id = net_id;
      t_wiring_via.layer_id = db.getLayerId("TOP");
      t_wiring_via.width = via.getSize();
      t_wiring_via.pt = via.getPos();
      wiring_vias.push_back(t_wiring_via);

      t_wiring_via.layer_id = db.getLayerId("BOTTOM");
      wiring_vias.push_back(t_wiring_via);
    }
  }

  // geometry from instances and components
  for (auto &inst : db.getInstances()) {
    int compId = inst.getComponentId();
    auto comp = db.getComponent(compId);

    int layer_id = inst.getLayer();
    // since we have the getPinPosition function
    // auto inst_angle = inst.getAngle();              // degree
    // point_2d inst_pos = {inst.getX(), inst.getY()}; // TODO: if coordinate to
    //                                                 // int
    auto &pin_net_map = inst.m_pin_net_map;
    // padstack
    for (auto &pad : comp.getPadstacks()) {

      metal_polygon t_metal_poly;
      t_metal_poly.layer_id = layer_id;

      auto &pad_name = pad.getName();
      auto net_id = (pin_net_map.find(pad_name))->second;
      if (net_id == -1) {
        net_id = ++max_net_id;
      }
      t_metal_poly.net_id = net_id;

      point_2d pin_pos; // absolute pin pos
      db.getPinPosition(pad, inst, &pin_pos);

      // get the coords of the padstack shapes
      auto coords = pad.getShapePolygon();
      for (auto &coord : coords) {
        coord = coord + pin_pos;
        t_metal_poly.pts.push_back(coord);
      }

      metal_polygons.push_back(t_metal_poly);
    }

    assert("circle is not supported yet" and comp.m_circles.size() == 0);
    assert("polygon is not supported yet" and comp.m_polys.size() == 0);
    assert("arc is not supported yet" and comp.m_arcs.size() == 0);
  }

  // geometry from orphan nets
  for (auto &net : db.orphan_nets) {
    int net_id = net.getId();
    if (net_id == -1) {
      net_id = ++max_net_id;
    }
    for (auto &wire : net.getSegments()) {
      wiring_wire t_wiring_wire;
      t_wiring_wire.net_id = net_id;
      t_wiring_wire.layer_id = db.getLayerId(wire.getLayer());
      t_wiring_wire.width = wire.getWidth();
      t_wiring_wire.seg = segment{wire.getPos().front(), wire.getPos().back()};
      wiring_wires.push_back(t_wiring_wire);
    }
    for (auto &via : net.getVias()) {
      wiring_via t_wiring_via;
      t_wiring_via.net_id = net_id;
      t_wiring_via.layer_id = db.getLayerId(
          "TOP"); // basically, via are thrupad , so we consider top and bottom
      t_wiring_via.width = via.getSize();
      t_wiring_via.pt = via.getPos();
      wiring_vias.push_back(t_wiring_via);

      t_wiring_via.layer_id = db.getLayerId("BOTTOM");
      wiring_vias.push_back(t_wiring_via);
    }
  }

  int top_layer_id = db.getLayerId("TOP");
  int bot_layer_id = db.getLayerId("BOTTOM");
  // init TOP and BOTTOM geometry
  for (auto &metal_polygon : metal_polygons) {
    if (metal_polygon.layer_id == top_layer_id) {
      top_metal_polygons.push_back(metal_polygon);
    }
    if (metal_polygon.layer_id == bot_layer_id) {
      bot_metal_polygons.push_back(metal_polygon);
    }
  }

  for (auto &wiring_wire : wiring_wires) {
    if (wiring_wire.layer_id == top_layer_id) {
      top_wiring_wires.push_back(wiring_wire);
    }
    if (wiring_wire.layer_id == bot_layer_id) {
      bot_wiring_wires.push_back(wiring_wire);
    }
  }

  for (auto &wiring_via : wiring_vias) {
    if (wiring_via.layer_id == top_layer_id) {
      top_wiring_vias.push_back(wiring_via);
    }
    if (wiring_via.layer_id == bot_layer_id) {
      bot_wiring_vias.push_back(wiring_via);
    }
  }

  // init the top_metal_segments and bot_metal_segments
  for (auto &metal_polygon : top_metal_polygons) {
    metal_segment t_metal_segment;
    t_metal_segment.net_id = metal_polygon.net_id;
    t_metal_segment.radius = 0;
    for (int i = 0; i < (int)metal_polygon.pts.size() - 1; i++) {
      t_metal_segment.seg =
          segment{metal_polygon.pts[i], metal_polygon.pts[i + 1]};
      top_metal_segments.push_back(t_metal_segment);
    }
  }
  for (auto &metal_polygon : bot_metal_polygons) {
    metal_segment t_metal_segment;
    t_metal_segment.net_id = metal_polygon.net_id;
    t_metal_segment.radius = 0;
    for (int i = 0; i < (int)metal_polygon.pts.size() - 1; i++) {
      t_metal_segment.seg =
          segment{metal_polygon.pts[i], metal_polygon.pts[i + 1]};
      bot_metal_segments.push_back(t_metal_segment);
    }
  }

  for (auto &wiring_wire : top_wiring_wires) {
    metal_segment t_metal_segment;
    t_metal_segment.net_id = wiring_wire.net_id;
    t_metal_segment.radius = wiring_wire.width / 2;
    t_metal_segment.seg = wiring_wire.seg;
    top_metal_segments.push_back(t_metal_segment);
  }
  for (auto &wiring_wire : bot_wiring_wires) {
    metal_segment t_metal_segment;
    t_metal_segment.net_id = wiring_wire.net_id;
    t_metal_segment.radius = wiring_wire.width / 2;
    t_metal_segment.seg = wiring_wire.seg;
    bot_metal_segments.push_back(t_metal_segment);
  }

  for (auto &wiring_via : top_wiring_vias) {
    metal_segment t_metal_segment;
    t_metal_segment.net_id = wiring_via.net_id;
    t_metal_segment.radius = wiring_via.width / 2;
    t_metal_segment.seg = segment{wiring_via.pt, wiring_via.pt};
    top_metal_segments.push_back(t_metal_segment);
  }
  for (auto &wiring_via : bot_wiring_vias) {
    metal_segment t_metal_segment;
    t_metal_segment.net_id = wiring_via.net_id;
    t_metal_segment.radius = wiring_via.width / 2;
    t_metal_segment.seg = segment{wiring_via.pt, wiring_via.pt};
    bot_metal_segments.push_back(t_metal_segment);
  }

}; // init the database from the dsnDataBase

void database::init_gpu_data() {
  for (auto &metal_segment : top_metal_segments) {
    gpu_seg_net_radius t_gpu_seg;
    // TODO: make pt0 and pt1 have order
    t_gpu_seg.x0 = metal_segment.seg.pt0.x() * layout_resolution;
    t_gpu_seg.y0 = metal_segment.seg.pt0.y() * layout_resolution;
    t_gpu_seg.x1 = metal_segment.seg.pt1.x() * layout_resolution;
    t_gpu_seg.y1 = metal_segment.seg.pt1.y() * layout_resolution;
    // if the segment is not in order, swap it
    if (t_gpu_seg.x0 > t_gpu_seg.x1) {
      std::swap(t_gpu_seg.x0, t_gpu_seg.x1);
      std::swap(t_gpu_seg.y0, t_gpu_seg.y1);
    }
    t_gpu_seg.net_id = metal_segment.net_id;
    // t_gpu_seg.radius = metal_segment.radius*layout_resolution;
    top_gpu_segs.push_back(t_gpu_seg);
  }

  for (auto &metal_segment : bot_metal_segments) {
    gpu_seg_net_radius t_gpu_seg;
    t_gpu_seg.x0 = metal_segment.seg.pt0.x() * layout_resolution;
    t_gpu_seg.y0 = metal_segment.seg.pt0.y() * layout_resolution;
    t_gpu_seg.x1 = metal_segment.seg.pt1.x() * layout_resolution;
    t_gpu_seg.y1 = metal_segment.seg.pt1.y() * layout_resolution;
    // if the segment is not in order, swap it
    if (t_gpu_seg.x0 > t_gpu_seg.x1) {
      std::swap(t_gpu_seg.x0, t_gpu_seg.x1);
      std::swap(t_gpu_seg.y0, t_gpu_seg.y1);
    }
    t_gpu_seg.net_id = metal_segment.net_id;
    // t_gpu_seg.radius = metal_segment.radius*layout_resolution;
    bot_gpu_segs.push_back(t_gpu_seg);
  }

}; // init the gpu

void database::modify_metal_segments() {
  std::vector<metal_segment> top_metal_segments_copy(top_metal_segments);
  std::vector<metal_segment> bot_metal_segments_copy(bot_metal_segments);

  top_metal_segments.clear();
  bot_metal_segments.clear();
  for (auto it : top_metal_segments_copy) {
    if (it.seg.pt0 == it.seg.pt1 and it.radius != 0) { // via
      metal_segment t;
      t.net_id = it.net_id;
      t.radius = 0;

      point_2d pt = it.seg.pt0, pt0 = point_2d{it.radius / 4, it.radius / 4},
               pt1 = point_2d{it.radius / 4, -it.radius / 4},
               pt2 = point_2d{-it.radius / 4, it.radius / 4},
               pt3 = point_2d{-it.radius / 4, -it.radius / 4};
      // for simplicity
      t.seg = segment{pt + pt0, pt + pt1};
      top_metal_segments.push_back(t);
      t.seg = segment{pt + pt0, pt + pt2};
      top_metal_segments.push_back(t);
      t.seg = segment{pt + pt3, pt + pt1};
      top_metal_segments.push_back(t);
      t.seg = segment{pt + pt3, pt + pt2};
      top_metal_segments.push_back(t);

    } else if (it.radius != 0) { // net
      metal_segment t;
      t.net_id = it.net_id;
      t.radius = 0;

      point_2d pt0 = point_2d{0, it.radius / 4},
               pt1 = point_2d{0, -it.radius / 4},
               pt2 = point_2d{0, it.radius / 4},
               pt3 = point_2d{0, -it.radius / 4};

      bool is_vertical = it.seg.pt0.x() == it.seg.pt1.x();
      if (not is_vertical) { // for simplicity
        point_2d pt_l = it.seg.pt0, pt_r = it.seg.pt1;
        if (pt_l.x() > pt_r.x()) {
          pt_l = it.seg.pt1, pt_r = it.seg.pt0;
        }
        t.seg = segment{pt_l + pt2, pt_l + pt0};
        top_metal_segments.push_back(t);
        t.seg = segment{pt_l + pt3, pt_r + pt1};
        top_metal_segments.push_back(t);
      }
    } else {
      top_metal_segments.push_back(it);
    }
  }

  for (auto it : bot_metal_segments_copy) {
    if (it.seg.pt0 == it.seg.pt1 and it.radius != 0) { // via
      metal_segment t;
      t.net_id = it.net_id;
      t.radius = 0;

      point_2d pt = it.seg.pt0, pt0 = point_2d{it.radius / 4, it.radius / 4},
               pt1 = point_2d{it.radius / 4, -it.radius / 4},
               pt2 = point_2d{-it.radius / 4, it.radius / 4},
               pt3 = point_2d{-it.radius / 4, -it.radius / 4};
      // for simplicity
      t.seg = segment{pt + pt0, pt + pt1};
      bot_metal_segments.push_back(t);
      t.seg = segment{pt + pt0, pt + pt2};
      bot_metal_segments.push_back(t);
      t.seg = segment{pt + pt3, pt + pt1};
      bot_metal_segments.push_back(t);
      t.seg = segment{pt + pt3, pt + pt2};
      bot_metal_segments.push_back(t);

    } else if (it.radius != 0) { // net
      metal_segment t;
      t.net_id = it.net_id;
      t.radius = 0;

      point_2d pt0 = point_2d{0, it.radius / 4},
               pt1 = point_2d{0, -it.radius / 4},
               pt2 = point_2d{0, it.radius / 4},
               pt3 = point_2d{0, -it.radius / 4};

      bool is_vertical = it.seg.pt0.x() == it.seg.pt1.x();
      if (not is_vertical) { // for simplicity
        point_2d pt_l = it.seg.pt0, pt_r = it.seg.pt1;
        if (pt_l.x() > pt_r.x()) {
          pt_l = it.seg.pt1, pt_r = it.seg.pt0;
        }
        t.seg = segment{pt_l + pt2, pt_l + pt0};
        bot_metal_segments.push_back(t);
        t.seg = segment{pt_l + pt3, pt_r + pt1};
        bot_metal_segments.push_back(t);
      }
    } else {
      bot_metal_segments.push_back(it);
    }
  }

  // get the largest width
  for (auto &metal_segment : top_metal_segments) {
    if (metal_segment.radius > largest_radius) {
      largest_radius = metal_segment.radius;
    }
  }
  for (auto &metal_segment : bot_metal_segments) {
    if (metal_segment.radius > largest_radius) {
      largest_radius = metal_segment.radius;
    }
  }

  for (auto it : top_metal_segments) {
    auto pt0 = it.seg.pt0;
    auto pt1 = it.seg.pt1;

    min_xy.m_x = std::min(min_xy.m_x, pt0.x());
    min_xy.m_x = std::min(min_xy.m_x, pt1.x());
    min_xy.m_y = std::min(min_xy.m_y, pt0.y());
    min_xy.m_y = std::min(min_xy.m_y, pt1.y());

    max_xy.m_x = std::max(max_xy.m_x, pt0.x());
    max_xy.m_x = std::max(max_xy.m_x, pt1.x());
    max_xy.m_y = std::max(max_xy.m_y, pt0.y());
    max_xy.m_y = std::max(max_xy.m_y, pt1.y());
  }
  for (auto it : bot_metal_segments) {
    auto pt0 = it.seg.pt0;
    auto pt1 = it.seg.pt1;

    min_xy.m_x = std::min(min_xy.m_x, pt0.x());
    min_xy.m_x = std::min(min_xy.m_x, pt1.x());
    min_xy.m_y = std::min(min_xy.m_y, pt0.y());
    min_xy.m_y = std::min(min_xy.m_y, pt1.y());

    max_xy.m_x = std::max(max_xy.m_x, pt0.x());
    max_xy.m_x = std::max(max_xy.m_x, pt1.x());
    max_xy.m_y = std::max(max_xy.m_y, pt0.y());
    max_xy.m_y = std::max(max_xy.m_y, pt1.y());
  }

}; //

void database::make2_2() {
  std::vector<metal_segment> top_metal_segments_copy(top_metal_segments);
  std::vector<metal_segment> bot_metal_segments_copy(bot_metal_segments);

  point_2d offset0 = max_xy - min_xy + point_2d{1, 1};
  point_2d offset1 = point_2d{0, offset0.y()};
  point_2d offset2 = point_2d{offset0.x(), 0};
  for (auto it : top_metal_segments_copy) {
    metal_segment t;
    t.net_id = it.net_id;
    t.radius = it.radius;
    t.seg = segment{it.seg.pt0 + offset0, it.seg.pt1 + offset0};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset1, it.seg.pt1 + offset1};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset2, it.seg.pt1 + offset2};
    top_metal_segments.push_back(it);
  }
  for (auto it : bot_metal_segments_copy) {
    metal_segment t;
    t.net_id = it.net_id;
    t.radius = it.radius;
    t.seg = segment{it.seg.pt0 + offset0, it.seg.pt1 + offset0};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset1, it.seg.pt1 + offset1};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset2, it.seg.pt1 + offset2};
    bot_metal_segments.push_back(it);
  }
}

void database::make4_2() {
  std::vector<metal_segment> top_metal_segments_copy(top_metal_segments);
  std::vector<metal_segment> bot_metal_segments_copy(bot_metal_segments);

  point_2d offset0 = max_xy - min_xy + point_2d{1, 1};
  point_2d offset1 = point_2d{0, offset0.y()};
  point_2d offset2 = point_2d{offset0.x(), 0};

  point_2d offset3 = offset2 + offset2;
  point_2d offset4 = offset0 + offset3;
  point_2d offset5 = offset1 + offset3;
  point_2d offset6 = offset2 + offset3;
  for (auto it : top_metal_segments_copy) {
    metal_segment t;
    t.net_id = it.net_id;
    t.radius = it.radius;
    t.seg = segment{it.seg.pt0 + offset0, it.seg.pt1 + offset0};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset1, it.seg.pt1 + offset1};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset2, it.seg.pt1 + offset2};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset3, it.seg.pt1 + offset3};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset4, it.seg.pt1 + offset4};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset5, it.seg.pt1 + offset5};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset6, it.seg.pt1 + offset6};
    top_metal_segments.push_back(it);
  }
  for (auto it : bot_metal_segments_copy) {
    metal_segment t;
    t.net_id = it.net_id;
    t.radius = it.radius;
    t.seg = segment{it.seg.pt0 + offset0, it.seg.pt1 + offset0};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset1, it.seg.pt1 + offset1};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset2, it.seg.pt1 + offset2};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset3, it.seg.pt1 + offset3};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset4, it.seg.pt1 + offset4};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset5, it.seg.pt1 + offset5};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset6, it.seg.pt1 + offset6};
    bot_metal_segments.push_back(it);
  }
}

void database::make4_4() {
  std::vector<metal_segment> top_metal_segments_copy(top_metal_segments);
  std::vector<metal_segment> bot_metal_segments_copy(bot_metal_segments);

  point_2d offset0 = max_xy - min_xy + point_2d{1, 1};
  point_2d offset1 = point_2d{0, offset0.y()};
  point_2d offset2 = point_2d{offset0.x(), 0};

  point_2d offset3 = offset2 + offset2;
  point_2d offset4 = offset0 + offset3;
  point_2d offset5 = offset1 + offset3;
  point_2d offset6 = offset2 + offset3;

  point_2d offset7 = offset1 + offset1;
  point_2d offset8 = offset0 + offset1;
  point_2d offset9 = offset1 + offset1;
  point_2d offset10 = offset2 + offset1;
  point_2d offset11 = offset3 + offset1;
  point_2d offset12 = offset4 + offset1;
  point_2d offset13 = offset5 + offset1;
  point_2d offset14 = offset6 + offset1;

  for (auto it : top_metal_segments_copy) {
    metal_segment t;
    t.net_id = it.net_id;
    t.radius = it.radius;
    t.seg = segment{it.seg.pt0 + offset0, it.seg.pt1 + offset0};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset1, it.seg.pt1 + offset1};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset2, it.seg.pt1 + offset2};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset3, it.seg.pt1 + offset3};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset4, it.seg.pt1 + offset4};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset5, it.seg.pt1 + offset5};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset6, it.seg.pt1 + offset6};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset7, it.seg.pt1 + offset7};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset8, it.seg.pt1 + offset8};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset9, it.seg.pt1 + offset9};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset10, it.seg.pt1 + offset10};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset11, it.seg.pt1 + offset11};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset12, it.seg.pt1 + offset12};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset13, it.seg.pt1 + offset13};
    top_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset14, it.seg.pt1 + offset14};
    top_metal_segments.push_back(it);
  }
  for (auto it : bot_metal_segments_copy) {
    metal_segment t;
    t.net_id = it.net_id;
    t.radius = it.radius;
    t.seg = segment{it.seg.pt0 + offset0, it.seg.pt1 + offset0};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset1, it.seg.pt1 + offset1};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset2, it.seg.pt1 + offset2};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset3, it.seg.pt1 + offset3};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset4, it.seg.pt1 + offset4};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset5, it.seg.pt1 + offset5};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset6, it.seg.pt1 + offset6};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset7, it.seg.pt1 + offset7};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset8, it.seg.pt1 + offset8};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset9, it.seg.pt1 + offset9};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset10, it.seg.pt1 + offset10};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset11, it.seg.pt1 + offset11};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset12, it.seg.pt1 + offset12};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset13, it.seg.pt1 + offset13};
    bot_metal_segments.push_back(it);
    t.seg = segment{it.seg.pt0 + offset14, it.seg.pt1 + offset14};
    bot_metal_segments.push_back(it);
  }
}

void database::print_metal_polygons() {
  for (auto &poly : metal_polygons) {
    std::cout << "net_id: " << poly.net_id << ", layer_id: " << poly.layer_id
              << ", #pts " << poly.pts.size() << std::endl;
    for (auto &pt : poly.pts) {
      std::cout << "(" << pt.x() << "," << pt.y() << ")"
                << " ";
    }
    std::cout << std::endl;
  }
};

void database::print_wiring_wires() {
  for (auto &wire : wiring_wires) {
    std::cout << "net_id: " << wire.net_id << ", layer_id: " << wire.layer_id
              << ", width: " << wire.width << ", seg: ";
    std::cout << "(" << wire.seg.pt0.x() << "," << wire.seg.pt0.y() << ")"
              << " ";
    std::cout << "(" << wire.seg.pt1.x() << "," << wire.seg.pt1.y() << ")"
              << " ";
    std::cout << std::endl;
  }
};

void database::print_wiring_vias() {
  for (auto &via : wiring_vias) {
    std::cout << "net_id: " << via.net_id << ", layer_id: " << via.layer_id
              << ", width: " << via.width << ", pt: ";
    std::cout << "(" << via.pt.x() << "," << via.pt.y() << ")"
              << " ";
    std::cout << std::endl;
  }
};

void database::print_top_metal_polygons() {
  for (auto &poly : top_metal_polygons) {
    std::cout << "net_id: " << poly.net_id << ", layer_id: " << poly.layer_id
              << ", #pts " << poly.pts.size() << std::endl;
    for (auto &pt : poly.pts) {
      std::cout << "(" << pt.x() << "," << pt.y() << ")"
                << " ";
    }
    std::cout << std::endl;
  }
};

void database::print_top_wiring_wires() {
  for (auto &wire : top_wiring_wires) {
    std::cout << "net_id: " << wire.net_id << ", layer_id: " << wire.layer_id
              << ", width: " << wire.width << ", seg: ";
    std::cout << "(" << wire.seg.pt0.x() << "," << wire.seg.pt0.y() << ")"
              << " ";
    std::cout << "(" << wire.seg.pt1.x() << "," << wire.seg.pt1.y() << ")"
              << " ";
    std::cout << std::endl;
  }
};

void database::print_top_wiring_vias() {
  for (auto &via : top_wiring_vias) {
    std::cout << "net_id: " << via.net_id << ", layer_id: " << via.layer_id
              << ", width: " << via.width << ", pt: ";
    std::cout << "(" << via.pt.x() << "," << via.pt.y() << ")"
              << " ";
    std::cout << std::endl;
  }
};

void database::print_bot_metal_polygons() {
  for (auto &poly : bot_metal_polygons) {
    std::cout << "net_id: " << poly.net_id << ", layer_id: " << poly.layer_id
              << ", #pts " << poly.pts.size() << std::endl;
    for (auto &pt : poly.pts) {
      std::cout << "(" << pt.x() << "," << pt.y() << ")"
                << " ";
    }
    std::cout << std::endl;
  }
};

void database::print_bot_wiring_wires() {
  for (auto &wire : bot_wiring_wires) {
    std::cout << "net_id: " << wire.net_id << ", layer_id: " << wire.layer_id
              << ", width: " << wire.width << ", seg: ";
    std::cout << "(" << wire.seg.pt0.x() << "," << wire.seg.pt0.y() << ")"
              << " ";
    std::cout << "(" << wire.seg.pt1.x() << "," << wire.seg.pt1.y() << ")"
              << " ";
    std::cout << std::endl;
  }
};

void database::print_bot_wiring_vias() {
  for (auto &via : bot_wiring_vias) {
    std::cout << "net_id: " << via.net_id << ", layer_id: " << via.layer_id
              << ", width: " << via.width << ", pt: ";
    std::cout << "(" << via.pt.x() << "," << via.pt.y() << ")"
              << " ";
    std::cout << std::endl;
  }
};

void database::init_info(dsnDataBase &db) {
  int num_top_components = 0, num_bot_components = 0;
  int num_top_padstacks = 0, num_bot_padstacks = 0;
  std::set<int> top_net, bot_net;

  for (auto &inst : db.getInstances()) {
    int compId = inst.getComponentId();
    auto comp = db.getComponent(compId);

    int layer_id = inst.getLayer();
    // since we have the getPinPosition function
    // auto inst_angle = inst.getAngle();              // degree
    // point_2d inst_pos = {inst.getX(), inst.getY()}; // TODO: if coordinate to
    //                                                 // int
    auto &pin_net_map = inst.m_pin_net_map;
    std::vector<int> nets;
    for (auto &it: pin_net_map){
      nets.push_back(it.second);
    }
    if (layer_id == db.getLayerId("TOP")) {
      num_top_components++;
      top_net.insert(nets.begin(), nets.end());
    } else if (layer_id == db.getLayerId("BOTTOM")) {
      num_bot_components++;
      bot_net.insert(nets.begin(), nets.end());
    }
    // padstack
    for (auto &pad : comp.getPadstacks()) {
      if (layer_id == db.getLayerId("TOP")) {
        num_top_padstacks++;
      } else if (layer_id == db.getLayerId("BOTTOM")) {
        num_bot_padstacks++;
      }
    }

    assert("circle is not supported yet" and comp.m_circles.size() == 0);
    assert("polygon is not supported yet" and comp.m_polys.size() == 0);
    assert("arc is not supported yet" and comp.m_arcs.size() == 0);
  }
  std::cout << "num_top_components: " << num_top_components << std::endl;
  std::cout << "num_bot_components: " << num_bot_components << std::endl;
  std::cout << "num_top_padstacks: " << num_top_padstacks << std::endl;
  std::cout << "num_bot_padstacks: " << num_bot_padstacks << std::endl;
  std::cout << "num_top_nets: " << top_net.size() << std::endl;
  std::cout << "num_bot_nets: " << bot_net.size() << std::endl;
}

} // namespace drc