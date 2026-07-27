/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 为什么是.cc文件：同样也是C++源文件，主要用于实现类的成员函数和其他功能。
 * （一个负责将单帧二维激光雷达数据“写入/更新”到二维概率栅格地图中的C++类。）这一部分是将激光雷达的点云数据插入到概率栅格地图中。它通过计算每个激光点的命中和未命中概率来更新栅格地图的占用状态。
 * 具体来说，它会根据激光点的位置和范围数据，确定哪些栅格单元被命中（即有障碍物）以及哪些栅格单元是空闲的（即没有障碍物）。然后，它会使用查找表来应用这些概率更新到栅格地图中，从而实现对环境的建模和感知。
 * RangeData：激光雷达的范围数据，包含激光雷达的原点、命中点和未命中点的点云信息。origin：激光雷达的原点位置，表示激光雷达在世界坐标系中的位置。returns：命中点的点云信息，表示激光雷达检测到的障碍物的位置。misses：未命中点的点云信息，表示激光雷达未检测到障碍物的位置。
 * ProbabilityGrid：二维概率栅格地图，用于表示环境中每个栅格单元的占用概率。
 * Submap：子地图，是对环境的一部分进行建模和表示的地图。每个子地图包含一个概率栅格地图和相关的位姿信息。
 * Free space：空闲空间，表示环境中没有障碍物的区域。在概率栅格地图中，空闲空间的栅格单元通常具有较低的占用概率。
 * Ray Casting：射线投射，是一种用于确定激光雷达扫描路径的方法。通过从激光雷达的原点向命中点和未命中点投射射线，可以确定哪些栅格单元被激光束穿过，从而更新这些栅格单元的占用状态。
 * GrowAsNeeded：根据激光雷达的范围数据，动态调整概率栅格地图的边界，以确保所有激光点都在地图范围内。
 * CastRays：根据激光雷达的范围数据，计算每条射线的命中和未命中栅格单元，并使用查找表更新概率栅格地图的占用状态。
 * FinishUpdate：完成一次更新操作，清理本轮更新的标记，以便下一次更新可以继续进行。
 *
*/

#include "cartographer/mapping/2d/probability_grid_range_data_inserter_2d.h"   //引入头文件，包含类的声明和相关依赖。

#include <cstdlib>

#include "Eigen/Core"  //数学库，提供矩阵和向量操作的功能。
#include "Eigen/Geometry" //数学库，提供几何操作的功能。
#include "cartographer/mapping/2d/xy_index.h"   //用于处理二维栅格地图中的索引和坐标转换。
#include "cartographer/mapping/internal/2d/ray_to_pixel_mask.h"  //用于将射线转换为栅格单元的掩码，以确定哪些栅格单元被射线穿过。
#include "cartographer/mapping/probability_values.h"  //用于处理概率值的相关函数和常量。
#include "glog/logging.h"   //Google的日志库，用于记录程序运行时的日志信息。

namespace cartographer {    //命名空间，避免命名冲突。
namespace mapping {
namespace {

// Factor for subpixel accuracy of start and end point for ray casts.
constexpr int kSubpixelScale = 1000;   //用于射线投射的亚像素精度因子，将栅格单元划分为更小的子像素，以提高射线投射的精度。

// 根据激光雷达的范围数据，动态调整概率栅格地图的边界，以确保所有激光点都在地图范围内。
void GrowAsNeeded(const sensor::RangeData& range_data,
                  ProbabilityGrid* const probability_grid) {
  Eigen::AlignedBox2f bounding_box(range_data.origin.head<2>());   //当前帧激光雷达的原点位置，因为是二维地图，所以只取前两个分量（x和y坐标）。创建一个新的矩形（边界框），先把机器人的当前位置 (x, y) 作为这个矩形的第一个（也是初始的）顶点放进去。
  // Padding around bounding box to avoid numerical issues at cell boundaries.
  constexpr float kPadding = 1e-6f;  //为了避免在栅格单元边界处出现数值问题，给边界框增加一个微小的填充量。这个填充量非常小（1e-6），用于确保在计算栅格单元索引时不会因为浮点数精度问题而导致错误的结果。
  // submap 初始只有 100x100 个格子。激光点超出当前边界时，先根据本帧 origin、
  // returns 和 misses 扩大地图，保证后续 ApplyLookupTable() 不会越界。
  for (const sensor::RangefinderPoint& hit : range_data.returns) {   //遍历激光雷达的命中点（returns），将每个命中点的位置加入到边界框中，以便计算出包含所有命中点的最小矩形区域。
    bounding_box.extend(hit.position.head<2>());
  }
  for (const sensor::RangefinderPoint& miss : range_data.misses) {   //遍历激光雷达的未命中点（misses），将每个未命中点的位置加入到边界框中，以便计算出包含所有未命中点的最小矩形区域。
    bounding_box.extend(miss.position.head<2>());
  }
  probability_grid->GrowLimits(bounding_box.min() -
                               kPadding * Eigen::Vector2f::Ones());  //根据计算出的边界框的最小点（左下角）减去填充量，调用概率栅格地图的 GrowLimits 方法，动态调整地图的边界，以确保所有激光点都在地图范围内。
  probability_grid->GrowLimits(bounding_box.max() +
                               kPadding * Eigen::Vector2f::Ones());  //根据计算出的边界框的最大点（右上角）加上填充量，调用概率栅格地图的 GrowLimits 方法，动态调整地图的边界，以确保所有激光点都在地图范围内。
}

// 根据激光雷达的范围数据，投射射线并更新概率栅格地图。负责把一帧激光雷达扫描数据（包括命中点和未命中点）插入到概率栅格地图中。它会根据每个激光点的位置，计算哪些栅格单元被命中（即有障碍物）以及哪些栅格单元是空闲的（即没有障碍物），并使用查找表来应用这些概率更新到栅格地图中。
void CastRays(const sensor::RangeData& range_data,
              const std::vector<uint16>& hit_table,
              const std::vector<uint16>& miss_table,
              const bool insert_free_space, ProbabilityGrid* probability_grid) {
  GrowAsNeeded(range_data, probability_grid);    //根据当前帧的激光雷达范围数据，动态调整概率栅格地图的边界，以确保所有激光点都在地图范围内。

  const MapLimits& limits = probability_grid->limits();   //获取当前概率栅格地图的边界信息，包括分辨率、最大坐标和栅格单元的数量等。
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;   //计算亚像素分辨率，将当前地图的分辨率除以亚像素精度因子（kSubpixelScale），用于射线投射时的更高精度计算。原本0.05m/格子，除以 1000 后变为 0.00005m/格子。
  const MapLimits superscaled_limits(        //创建一个新的 MapLimits 对象，表示亚像素级别的地图边界信息。这个新的边界信息用于射线投射时的计算。
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =                //计算射线的起点索引，即激光雷达原点在亚像素级别地图中的栅格单元索引。通过调用 superscaled_limits.GetCellIndex 方法，将激光雷达的原点位置（range_data.origin）转换为对应的栅格单元索引。由于是二维地图，只取原点的前两个分量（x 和 y 坐标）。
      superscaled_limits.GetCellIndex(range_data.origin.head<2>());
  // Compute and add the end points.
  // 先处理 hit 末端：障碍物格子优先被标记为占用。下面暂不 FinishUpdate()，
  // 因此同一帧里如果射线又穿过同一格，miss 不会覆盖 hit。
  std::vector<Eigen::Array2i> ends;      //墙的优先级必须高于空闲空间的优先级。因为如果一束 scan 里同一格同时被多条射线经过或命中，update marker 可避免单帧内重复累加导致概率跳变过猛。
  ends.reserve(range_data.returns.size());
  for (const sensor::RangefinderPoint& hit : range_data.returns) {
    ends.push_back(superscaled_limits.GetCellIndex(hit.position.head<2>()));
    probability_grid->ApplyLookupTable(ends.back() / kSubpixelScale, hit_table);
  }

  if (!insert_free_space) {
    return;
  }

  // Now add the misses.
  // 再处理每条命中射线起点到终点之间的空闲格。RayToPixelMask 返回的是穿过的
  // 栅格序列，miss_table 会降低这些格子的占用概率，所以可视化上逐渐变白。
  for (const Eigen::Array2i& end : ends) {
    std::vector<Eigen::Array2i> ray =
        RayToPixelMask(begin, end, kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }

  // Finally, compute and add empty rays based on misses in the range data.
  // range_data.misses 代表超过 max_range 或无回波方向的“空射线”。它们没有障碍
  // 末端，只把射线路径标为空闲，用于清理可通行区域。   处理未命中射线的空闲格子。对于每个未命中点，计算从激光雷达原点到该点的射线路径，并将路径上的栅格单元标记为空闲（降低占用概率）。
  for (const sensor::RangefinderPoint& missing_echo : range_data.misses) {
    std::vector<Eigen::Array2i> ray = RayToPixelMask(
        begin, superscaled_limits.GetCellIndex(missing_echo.position.head<2>()),
        kSubpixelScale);
    for (const Eigen::Array2i& cell_index : ray) {
      probability_grid->ApplyLookupTable(cell_index, miss_table);
    }
  }
}
}  // namespace

proto::ProbabilityGridRangeDataInserterOptions2D   //概率栅格地图数据插入器选项类，包含命中概率、未命中概率和是否插入空闲空间的设置。
CreateProbabilityGridRangeDataInserterOptions2D(
    common::LuaParameterDictionary* parameter_dictionary) {
  proto::ProbabilityGridRangeDataInserterOptions2D options;
  options.set_hit_probability(
      parameter_dictionary->GetDouble("hit_probability"));
  options.set_miss_probability(
      parameter_dictionary->GetDouble("miss_probability"));
  options.set_insert_free_space(
      parameter_dictionary->HasKey("insert_free_space")
          ? parameter_dictionary->GetBool("insert_free_space")
          : true);
  CHECK_GT(options.hit_probability(), 0.5);
  CHECK_LT(options.miss_probability(), 0.5);
  return options;
}

ProbabilityGridRangeDataInserter2D::ProbabilityGridRangeDataInserter2D(
    const proto::ProbabilityGridRangeDataInserterOptions2D& options)
    : options_(options),
      hit_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.hit_probability()))),
      miss_table_(ComputeLookupTableToApplyCorrespondenceCostOdds(
          Odds(options.miss_probability()))) {}

void ProbabilityGridRangeDataInserter2D::Insert(     //真正的插入函数，将一帧激光雷达的范围数据插入到概率栅格地图中。它会根据每个激光点的位置，计算哪些栅格单元被命中（即有障碍物）以及哪些栅格单元是空闲的（即没有障碍物），并使用查找表来应用这些概率更新到栅格地图中。
    const sensor::RangeData& range_data, GridInterface* const grid) const {
  ProbabilityGrid* const probability_grid = static_cast<ProbabilityGrid*>(grid);
  CHECK(probability_grid != nullptr);
  // By not finishing the update after hits are inserted, we give hits priority
  // (i.e. no hits will be ignored because of a miss in the same cell).
  // 一次 Insert 对应一批已经匹配到 local frame 的 scan。CastRays 完成 hit/miss
  // 累积后，FinishUpdate() 清理本轮 marker，使下一批 scan 可以继续更新这些格子。
  CastRays(range_data, hit_table_, miss_table_, options_.insert_free_space(),
           probability_grid);
  probability_grid->FinishUpdate();   //完成一次更新操作，清理本轮更新的标记，以便下一次更新可以继续进行。
}

}  // namespace mapping
}  // namespace cartographer
