#pragma once
#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <cuda_runtime.h>
// TODO: rewrite it
template <typename T> __host__ __device__ int dcmp(T a) {
  return a < 0 ? -1 : (a > 0 ? 1 : 0);
}

template <typename T>
__host__ __device__ T crossProduct(T x0, T y0, T x1, T y1) {
  return x0 * y1 - y0 * x1;
}

template <typename T>
__host__ __device__ bool isIntersection(T x0, T y0, T x1, T y1, T x2, T y2,
                                        T x3, T y3) {
  T c1 = crossProduct(x1 - x0, y1 - y0, x2 - x0, y2 - y0),
    c2 = crossProduct(x1 - x0, y1 - y0, x3 - x0, y3 - y0);
  T d1 = crossProduct(x3 - x2, y3 - y2, x0 - x2, y0 - y2),
    d2 = crossProduct(x3 - x2, y3 - y2, x1 - x2, y1 - y2);

  return dcmp(c1) * dcmp(c2) < 0 and dcmp(d1) * dcmp(d2) < 0; // 分别在两侧
}

template <typename T>
__host__ __device__ void getIntersection(T ax, T ay, T bx, T by, T cx, T cy,
                                         T dx, T dy, T &res_x, T &res_y) {
  T xx = bx - ax, xy = by - ay, yx = dx - cx, yy = dy - cy, zx = ax - cx,
    zy = ay - cy;
  res_x = xx * crossProduct(yx, yy, zx, zy) / crossProduct(xx, xy, yx, yy) + ax;
  res_y = xy * crossProduct(yx, yy, zx, zy) / crossProduct(xx, xy, yx, yy) + ay;
}

#endif