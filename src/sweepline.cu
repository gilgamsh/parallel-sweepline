#include "geometry.cuh"
#include "sweepline.cuh"
#include "sweepline.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <thrust/async/sort.h>
#include <thrust/device_reference.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/scan.h>
#include <thrust/unique.h>

#include <cub/cub.cuh>

/*
1. build the global x-axis datastructure
2. each block will build the local y-axis coordinate for each pos(event point),
and check the neighbor(of event point)
*/
// TODO: maybe use a heap to store the tmp smallest right endpoints and
// according to this get the hierarch interval lists
/*
1. each time update the value of the heap(except for the same),pop and insert
the new right endpoints.(saying that you always insert the new interval into the
lists have the smallest right endpoints)
2. if you cann't pop , just insert.(create a new list)

*/
namespace drc {

struct gpu_event {
  int event;
  int seg_id;
  bool is_start;
};

struct hint {
  gpu_seg_net_radius *d_hint;
  int size;
};

__global__ void compute_seg_hier(gpu_seg_net_radius *d_segs, int size,
                                 int *d_seg_hier) {
  int tid = threadIdx.x + blockDim.x * blockIdx.x;
  if (tid >= size)
    return;
  int hier = 0;
  int length = d_segs[tid].x1 - d_segs[tid].x0;
  length >>= 1;
  while (length) {
    length >>= 1;
    hier++;
  }
  d_seg_hier[tid] = hier;
}

/*
1. scan the multiple hints and compute the intersection
2. use cub do blockwise parallel sort
3. according to the event, find the neightbor segment exchange
TODO: try to use cudaMemcpyAsync to merge the data to global memory and use cub
do parallel sort and compare
*/



__global__ void build_neighbor(int *d_pos, int num_pos, hint *d_hints,
                               int hint_size, gpu_seg_net_radius *sort_buffers,
                               int *sort_intersection_buffers) {
  int bid = blockIdx.x;
  int tid = threadIdx.x;

  extern __shared__ int s[];
  // with hint_size hints_l[i] is for i hierarchy hint left point
  int no_hints = tid; // each thread compute one hint with the same hierarchy

  while (no_hints < hint_size) {
    if (d_hints[no_hints].size <= 0) {
      s[no_hints] = 0;
      s[no_hints + hint_size] = -1;
      no_hints += blockDim.x;
      continue;
    }
    {
      // compute the right point first ,check(mid) will be mid point be the <=
      // pos
      int l = 0;
      int r = d_hints[no_hints].size - 1;
      int mid;
      auto hint = d_hints[no_hints].d_hint;

      while (l < r) {
        mid = (l + r + 1) >> 1;
        if (hint[mid].x0 <= d_pos[bid]) {
          l = mid;
        } else {
          r = mid - 1;
        }
      }
      s[no_hints + hint_size] = l;
    }
    {
      // compute the left point, check(mid) will be mid point>=
      // pos-length_of_hierarchy
      int l = 0;
      int r = d_hints[no_hints].size - 1;
      int mid;
      auto hint = d_hints[no_hints].d_hint;
      int length = 1 << (no_hints + 1);
      length--;

      while (l < r) {
        mid = (l + r) >> 1;
        if (hint[mid].x0 >= d_pos[bid] - length) {
          r = mid;
        } else {
          l = mid + 1;
        }
      }
      s[no_hints] = l;
      if (hint[l].x0 > d_pos[bid]) {
        s[no_hints + hint_size] = l - 1;
      }
    }
    no_hints += blockDim.x;
  }

  __syncthreads();

  // finish the multi-list scan , compute the prefix sum and move data
  int prefix_sum_start = hint_size * 2 + 1;
  if (tid == 0) { // TODO： look like prefix sum
    s[prefix_sum_start - 1] = 0;
    s[prefix_sum_start] = s[hint_size + 0] - s[0] + 1;
    for (int i = 1; i < hint_size; i++) {
      s[prefix_sum_start + i] =
          s[prefix_sum_start + i - 1] + s[hint_size + i] - s[i] + 1;
    }
  }

  __syncthreads();

  // move the data to the right place
  int no_hints_trans = tid;
  while (no_hints_trans < hint_size) {
    int l = s[no_hints_trans];
    int r = s[no_hints_trans + hint_size];
    int length = r - l + 1;

    int offset =
        s[prefix_sum_start + no_hints_trans - 1] + bid * max_num_per_line;
    auto hint = d_hints[no_hints_trans].d_hint;

    for (int i = 0; i < length; i++) {
      sort_buffers[i + offset] = *(hint + i + l);
      // compute the intersection
      int intersection = -1;
      int x0 = hint[i + l].x0;
      int x1 = hint[i + l].x1;
      int y0 = hint[i + l].y0;
      int y1 = hint[i + l].y1;
      if (x0 <= d_pos[bid] and x1 >= d_pos[bid]) {
        if (x1 == x0) {
          intersection = y0;
        } else {
          int left = d_pos[bid] - x0;
          intersection = y0 + (y1 - y0) * left / (x1 - x0);
        }
      }
      sort_intersection_buffers[i + offset] = intersection;
    }
    no_hints_trans += blockDim.x;
  }
}

template <typename Key, int Block_Threads, int Items_Per_Thread,
          typename ValueT>
__launch_bounds__(Block_Threads) __global__
    void sort_neighbour(Key *d_key, ValueT *d_value) {

  using namespace cub;
  enum { Tile_Size = Block_Threads * Items_Per_Thread };

  using BlockLoadKeyT = BlockLoad<Key, Block_Threads, Items_Per_Thread,
                                  BLOCK_LOAD_WARP_TRANSPOSE>;
  using BlockLoadValueT = BlockLoad<ValueT, Block_Threads, Items_Per_Thread,
                                    BLOCK_LOAD_WARP_TRANSPOSE>;
  using BlockRadixSortT =
      BlockRadixSort<Key, Block_Threads, Items_Per_Thread, ValueT>;

  __shared__ union TempStorage {
    typename BlockLoadKeyT::TempStorage load_key;
    typename BlockLoadValueT::TempStorage load_value;
    typename BlockRadixSortT::TempStorage sort;
  } temp_storage;

  Key keys[Items_Per_Thread];
  ValueT values[Items_Per_Thread];

  int block_offset = blockIdx.x * Tile_Size;

  BlockLoadKeyT(temp_storage.load_key).Load(d_key + block_offset, keys);

  __syncthreads();

  BlockLoadValueT(temp_storage.load_value).Load(d_value + block_offset, values);

  __syncthreads();

  BlockRadixSortT(temp_storage.sort).SortBlockedToStriped(keys, values);

  StoreDirectStriped<Block_Threads>(threadIdx.x, d_key + block_offset, keys);
  StoreDirectStriped<Block_Threads>(threadIdx.x, d_value + block_offset,
                                    values);
}

__global__ void insert_d_pos_set(int *d_pos, int num_pos, int *d_pos_set) {
  int tid = threadIdx.x + blockIdx.x * blockDim.x;
  if (tid >= num_pos)
    return;
  insert(d_pos_set, d_pos[tid]);
}

template <typename Key, typename ValueT>
__global__ void check_neighbour(Key *d_key, ValueT *d_value, int *d_pos,
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
    bool flag0 = (x0 == d_pos[bid]) or (x1 == d_pos[bid]);
    bool flag1 = (x0_l == d_pos[bid]) or (x1_l == d_pos[bid]);
    if (not(flag0 or flag1)) {
      check_id += blockDim.x;
      continue;
    }

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

void sweepline_check(std::vector<gpu_seg_net_radius> &segs) {

  using namespace std::chrono;

  cudaError_t err;

  cudaStream_t stream_segs = nullptr;
  cudaStream_t stream_event = nullptr;
  cudaStream_t stream_pos = nullptr;

  cudaStreamCreate(&stream_segs);
  cudaStreamCreate(&stream_event);
  cudaStreamCreate(&stream_pos);

  gpu_seg_net_radius *d_segs;
  int *d_seg_hier; // to compute the hier
  cudaMallocAsync((void **)&d_segs, sizeof(gpu_seg_net_radius) * segs.size(),
                  stream_segs);
  cudaMemcpyAsync(d_segs, segs.data(), sizeof(gpu_seg_net_radius) * segs.size(),
                  cudaMemcpyHostToDevice, stream_segs);
  cudaMallocAsync((void **)&d_seg_hier, sizeof(int) * segs.size(), stream_segs);

  hint *d_hints;
  std::vector<hint> h_hints; // hints mean they have the same hier
  // construct hints
  {
    constexpr int bs = compute_seg_hier_bs;
    compute_seg_hier<<<(segs.size() + bs - 1) / bs, bs, 0, stream_segs>>>(
        d_segs, segs.size(), d_seg_hier);

    std::vector<int> h_seg_hier(segs.size());
    cudaMemcpyAsync(h_seg_hier.data(), d_seg_hier,
                    sizeof(int) * h_seg_hier.size(), cudaMemcpyDeviceToHost,
                    stream_segs);

    cudaStreamSynchronize(stream_segs);

    std::map<int, int> hier_count;
    for (auto &h : h_seg_hier) {
      hier_count[h]++;
    }
    int max_hier = 0;
    for (auto &p : hier_count) {
      max_hier = std::max(max_hier, p.first);
    }

    h_hints.resize(max_hier + 1);
    for (auto &p : hier_count) {
      cudaMallocAsync((void **)&h_hints[p.first].d_hint,
                      sizeof(gpu_seg_net_radius) * p.second, stream_segs);
      h_hints[p.first].size = p.second;
    }

    std::vector<std::vector<gpu_seg_net_radius>> seg_hints(
        max_hier + 1, std::vector<gpu_seg_net_radius>());
    for (int i = 0; i < (int)h_seg_hier.size(); i++) {
      seg_hints[h_seg_hier[i]].push_back(segs[i]);
    }

    for (int i = 0; i < (int)seg_hints.size(); i++) {
      if (seg_hints[i].empty())
        continue;

      cudaMemcpyAsync(h_hints[i].d_hint, seg_hints[i].data(),
                      sizeof(gpu_seg_net_radius) * seg_hints[i].size(),
                      cudaMemcpyHostToDevice, stream_segs);

      thrust::async::sort(
          thrust::cuda::par.on(stream_segs), h_hints[i].d_hint,
          h_hints[i].d_hint + seg_hints[i].size(),
          [] __device__(const gpu_seg_net_radius &a,
                        const gpu_seg_net_radius &b) { return a.x0 < b.x0; });
    }

    cudaMallocAsync((void **)&d_hints, sizeof(hint) * h_hints.size(),
                    stream_segs);
    cudaMemcpyAsync(d_hints, h_hints.data(), sizeof(hint) * h_hints.size(),
                    cudaMemcpyHostToDevice, stream_segs);
  }

  std::vector<int> pos;
  // std::vector<gpu_event> events;

  for (int i = 0; i < (int)segs.size(); i++) {
    auto &seg = segs[i];
    pos.push_back(seg.x0);
    pos.push_back(seg.x1);
    // events.emplace_back(gpu_event{seg.x0, i, true});
    // events.emplace_back(gpu_event{seg.x1, i, false});
  }

  int *d_pos;
  // gpu_event *d_events;

  cudaMallocAsync((void **)&d_pos, sizeof(int) * pos.size(), stream_pos);
  // cudaMallocAsync((void **)&d_events, sizeof(gpu_event) * events.size(),
  //                 stream_event);
  cudaMemcpyAsync(d_pos, pos.data(), sizeof(int) * pos.size(),
                  cudaMemcpyHostToDevice, stream_pos);
  // cudaMemcpyAsync(d_events, events.data(), sizeof(gpu_event) * events.size(),
  //                 cudaMemcpyHostToDevice, stream_event);

  thrust::async::sort(
      thrust::cuda::par.on(stream_pos), d_pos, d_pos + pos.size(),
      [] __device__(const int a, const int b) { return a < b; });

  

  auto end_pos = thrust::unique(thrust::cuda::par.on(stream_pos), d_pos,
                                d_pos + pos.size());
  int num_pos = end_pos - d_pos;

  gpu_seg_net_radius *sort_buffers = nullptr, *iter_sort_buffers = nullptr;
  int *sort_intersection_buffers = nullptr,
      *iter_sort_intersection_buffers = nullptr;
  int *d_pos_set = nullptr, *d_pos_new = nullptr, *d_pos_new_id = nullptr;

  cudaMallocAsync((void **)&sort_buffers,
                  sizeof(gpu_seg_net_radius) * num_pos * max_num_per_line,
                  stream_event);
  cudaMallocAsync((void **)&sort_intersection_buffers,
                  sizeof(int) * num_pos * max_num_per_line, stream_event);
  cudaMemsetAsync(sort_intersection_buffers, -1,
                  sizeof(int) * num_pos * max_num_per_line, stream_event);

  cudaMallocAsync((void **)&iter_sort_buffers,
                  sizeof(gpu_seg_net_radius) * num_new_events *
                      max_num_per_line,
                  stream_pos);
  cudaMallocAsync((void **)&iter_sort_intersection_buffers,
                  sizeof(int) * num_new_events * max_num_per_line, stream_pos);

  cudaMallocAsync((void **)&d_pos_set, sizeof(int) * num_d_sets, stream_pos);
  cudaMemsetAsync(d_pos_set, -1, sizeof(int) * num_d_sets, stream_pos);

  cudaMallocAsync((void **)&d_pos_new, sizeof(int) * num_new_events,
                  stream_pos);
  cudaMemsetAsync(d_pos_new, 0x7f, sizeof(int) * num_new_events, stream_pos);
  cudaMallocAsync((void **)&d_pos_new_id, sizeof(int) * num_new_events,
                  stream_pos);

  { // init_d_pos_set
    constexpr int bs = insert_d_pos_set_bs;
    insert_d_pos_set<<<(num_pos + bs - 1) / bs, bs, 0, stream_pos>>>(
        d_pos, num_pos, d_pos_set);
  }

  { // build neighor for each pos
    constexpr int bs = build_neighbor_bs;
    build_neighbor<<<num_pos, bs, sizeof(int) * (3 * h_hints.size() + 1),
                     stream_event>>>(d_pos, num_pos, d_hints, h_hints.size(),
                                     sort_buffers, sort_intersection_buffers);
  }
  { // sort and compare for each pos
    constexpr int bs = sort_and_compare_bs;
    sort_neighbour<int, bs, max_num_per_line / bs, gpu_seg_net_radius>
        <<<num_pos, bs>>>(sort_intersection_buffers, sort_buffers);
    check_neighbour<int, gpu_seg_net_radius><<<num_pos, bs>>>(
        sort_intersection_buffers, sort_buffers, d_pos, d_pos_set, d_pos_new);
  }

  { // iterative check
    int *first_pos;
    cudaMallocHost((void **)&first_pos, sizeof(int));
    thrust::async::sort(
        thrust::cuda::par.on(stream_event), d_pos_new,
        d_pos_new + num_new_events,
        [] __device__(const int a, const int b) { return a < b; });
    auto end_pos = thrust::unique(thrust::cuda::par.on(stream_event), d_pos_new,
                                  d_pos_new + num_new_events);
    int num_new_pos = end_pos - d_pos_new;

    cudaMemcpy(first_pos, d_pos_new, sizeof(int), cudaMemcpyDeviceToHost);

    while (*first_pos < 0x7f7f7f7f) {
      {
        auto bs = build_iter_neighbor_bs;
        build_iter_neighbor<<<(num_new_pos + bs - 1), bs, 0, stream_pos>>>(
            d_pos, num_pos, d_pos_new, num_new_pos, d_pos_new_id);
      }
      {
        auto bs = insert_d_pos_set_bs;
        insert_d_pos_set<<<(num_new_pos + bs - 1) / bs, bs, 0, stream_pos>>>(
            d_pos_new, num_new_pos, d_pos_set);
      }
      {
        auto bs = sort_and_compare_bs;
        sort_iter_neighbor<<<num_new_pos, bs>>>(
            sort_buffers, iter_sort_buffers, iter_sort_intersection_buffers,
            d_pos_new, num_new_pos, d_pos_new_id);
      }
      cudaMemsetAsync(d_pos_new, 0x7f, sizeof(int) * num_new_events,
                      stream_pos);
      {
        auto bs = sort_and_compare_bs;
        check_iter_neighbour<<<num_new_pos, bs>>>(
            iter_sort_intersection_buffers, iter_sort_buffers, d_pos_set,
            d_pos_new);
      }

      cudaDeviceSynchronize();
      

      thrust::async::sort(
          thrust::cuda::par.on(stream_pos), d_pos_new,
          d_pos_new + num_new_events,
          [] __device__(const int a, const int b) { return a < b; });
      auto end_pos = thrust::unique(thrust::cuda::par.on(stream_pos), d_pos_new,
                                    d_pos_new + num_new_events);
      num_new_pos = end_pos - d_pos_new;
      cudaMemcpy(first_pos, d_pos_new, sizeof(int), cudaMemcpyDeviceToHost);
    }
  }

  cudaDeviceSynchronize();
}

} // namespace drc

