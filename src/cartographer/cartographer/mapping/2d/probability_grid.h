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
 */

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <vector>

#include "cartographer/common/port.h"
#include "cartographer/mapping/2d/grid_2d.h"
#include "cartographer/mapping/2d/map_limits.h"
#include "cartographer/mapping/2d/xy_index.h"

namespace cartographer {
namespace mapping {

// Represents a 2D grid of probabilities.
//
// 中文导读：
// ProbabilityGrid 是 Cartographer 2D 常用的占据栅格。内部实际存的是
// correspondence cost 的离散值，但对外提供 GetProbability()/SetProbability()
// 这样的概率接口。直观上：
//   - 激光命中 returns 的末端：占用概率增加，RViz/纹理里会逐渐变黑；
//   - 激光穿过的 free space 或 misses：占用概率降低，显示会逐渐变白；
//   - 从未观测过的格子保持 unknown，不应被误当作空闲。
class ProbabilityGrid : public Grid2D {
 public:
  explicit ProbabilityGrid(const MapLimits& limits,
                           ValueConversionTables* conversion_tables);
  explicit ProbabilityGrid(const proto::Grid2D& proto,
                           ValueConversionTables* conversion_tables);

  // Sets the probability of the cell at 'cell_index' to the given
  // 'probability'. Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& cell_index,
                      const float probability);

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'cell_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // FinishUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  //
  // 注意“同一次 Insert 内每格只更新一次”：一束 scan 里同一格可能同时被多条射线
  // 经过或命中，update marker 可避免单帧内重复累加导致概率跳变过猛。
  bool ApplyLookupTable(const Eigen::Array2i& cell_index,
                        const std::vector<uint16>& table);

  GridType GetGridType() const override;

  // Returns the probability of the cell with 'cell_index'.
  float GetProbability(const Eigen::Array2i& cell_index) const;

  proto::Grid2D ToProto() const override;
  std::unique_ptr<Grid2D> ComputeCroppedGrid() const override;
  bool DrawToSubmapTexture(
      proto::SubmapQuery::Response::SubmapTexture* const texture,
      transform::Rigid3d local_pose) const override;

 private:
  ValueConversionTables* conversion_tables_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
