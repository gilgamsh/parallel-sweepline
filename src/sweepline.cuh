#pragma once
#ifndef SWEEP_CUH
#define SWEEP_CUH
#include "drc_database.hpp"
#include "geometry.cuh"
#include <cub/cub.cuh>
namespace drc {

__device__ int gpu_hash(int x) {
  x = x % num_d_sets;
  return x;
}

// all threads can only write distinct values
__device__ void insert(int *d_pos_set, int pos) {
  int hash_v = gpu_hash(pos);
  while (not atomicCAS(d_pos_set + hash_v, -1, pos)) {
    hash_v++;
    hash_v %= num_d_sets;
  }
}

// all threads can only read
__device__ bool contain(int *d_pos_set, int pos) {
  int hash_v = gpu_hash(pos);
  while (d_pos_set[hash_v] != -1) {
    if (d_pos_set[hash_v] == pos) {
      return true;
    }
    hash_v++;
    hash_v %= num_d_sets;
  }
  return false;
}

/*
1. find the corresponding neighbor of each pos with sort_buffers and d_pos
2. recompute the intersection and restore the result to iter_sort_buffer
3. check neighbour
*/
__global__ void build_iter_neighbor(int *d_pos, int num_pos, int *d_pos_new,
                                    int num_new_pos, int *d_pos_new_id) {
  int id = blockIdx.x * blockDim.x + threadIdx.x;
  if (id >= num_new_pos)
    return;
  int new_pos = d_pos_new[id];

  int l = 0;
  int r = num_pos - 1;
  while (l < r) { // find the last that new_pos >  d_pos[mid], d_pos will
                  // never contain new_pos
    int mid = (l + r + 1) / 2;
    if (new_pos > d_pos[mid])
      l = mid;
    else
      r = mid - 1;
  }
  d_pos_new_id[id] = l;
}

__global__ void sort_iter_neighbor(gpu_seg_net_radius *sort_buffers,
                                   gpu_seg_net_radius *iter_sort_buffers,
                                   int *iter_sort_intersection_buffers,
                                   int *d_pos_new, int num_new_pos,
                                   int *d_pos_new_id) {
  int bid = blockIdx.x;
  int tid = threadIdx.x;
  if (bid >= num_new_pos)
    return;
  int new_pos = d_pos_new[bid];
  int block_offset = d_pos_new_id[bid] * max_num_per_line;
  //   int tid_trans = tid + d_pos_new_id[bid] * max_num_per_line;
  using namespace cub;

  constexpr int Block_Threads = sort_and_compare_bs;
  constexpr int Items_Per_Thread = max_num_per_line / Block_Threads;

  using BlockLoadValueT =
      BlockLoad<gpu_seg_net_radius, Block_Threads, Items_Per_Thread,
                BLOCK_LOAD_WARP_TRANSPOSE>;
  using BlockRadixSortT =
      BlockRadixSort<int, Block_Threads, Items_Per_Thread, gpu_seg_net_radius>;

  __shared__ union TempStorage {
    typename BlockLoadValueT::TempStorage load;
    typename BlockRadixSortT::TempStorage sort;
  } temp_storage;

  int keys[Items_Per_Thread];
  gpu_seg_net_radius values[Items_Per_Thread];

  BlockLoadValueT(temp_storage.load).Load(sort_buffers + block_offset, values);

  __syncthreads();

  for (int i = 0; i < Items_Per_Thread; i++) {
    int intersection = -1;
    int x0 = values[i].x0;
    int x1 = values[i].x1;
    int y0 = values[i].y0;
    int y1 = values[i].y1;
    if (x0 <= new_pos and new_pos <= x1) {
      if (x1 == x0) {
        intersection = y0;
      } else {
        intersection = y0 + (y1 - y0) * (new_pos - x0) / (x1 - x0);
      }
    }
    keys[i] = intersection;
  }

  __syncthreads();

  BlockRadixSortT(temp_storage.sort).SortBlockedToStriped(keys, values);

  StoreDirectStriped<Block_Threads>(tid, iter_sort_buffers + block_offset,
                                    values);
  StoreDirectStriped<Block_Threads>(
      tid, iter_sort_intersection_buffers + block_offset, keys);
}

__global__ void check_iter_neighbour(int *d_key, gpu_seg_net_radius *d_value,
                                     int *d_pos_set, int *d_pos_new) {
  int tid = threadIdx.x;
  int bid = blockIdx.x;
  int block_offset = bid * max_num_per_line;

  int check_id = tid + 1;
  while (check_id < max_num_per_line) {
    int left_id = check_id - 1;
    if (d_key[check_id + block_offset] <= -1 or
        d_key[left_id + block_offset] <= -1) {
      check_id += blockDim.x;
      continue;
    }
    int x0, x0_l;
    int x1, x1_l;

    x0 = d_value[check_id + block_offset].x0;
    x1 = d_value[check_id + block_offset].x1;
    x0_l = d_value[left_id + block_offset].x0;
    x1_l = d_value[left_id + block_offset].x1;

    int y0, y0_l;
    int y1, y1_l;
    y0 = d_value[check_id + block_offset].y0;
    y1 = d_value[check_id + block_offset].y1;
    y0_l = d_value[left_id + block_offset].y0;
    y1_l = d_value[left_id + block_offset].y1;

    bool is_intersection =
        isIntersection(x0, y0, x1, y1, x0_l, y0_l, x1_l, y1_l);
    if (is_intersection and d_value[check_id + block_offset].net_id !=
                                d_value[left_id + block_offset].net_id) {
      int x_intersection, y_intersection;
      getIntersection(x0, y0, x1, y1, x0_l, y0_l, x1_l, y1_l, x_intersection,
                      y_intersection);
      if (not contain(d_pos_set, x_intersection)) {
        int id = threadIdx.x + blockIdx.x * blockDim.x;
        id %= num_new_events;
        while (not atomicCAS(d_pos_new + id, 0x7f7f7f7f, x_intersection)) {
          id++;
          id %= num_new_events;
        }
      }
    }

    check_id += blockDim.x;
  }
}

} // namespace drc

#endif