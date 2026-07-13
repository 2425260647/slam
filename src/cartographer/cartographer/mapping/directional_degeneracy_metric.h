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

#ifndef CARTOGRAPHER_MAPPING_DIRECTIONAL_DEGENERACY_METRIC_H_
#define CARTOGRAPHER_MAPPING_DIRECTIONAL_DEGENERACY_METRIC_H_

#include "Eigen/Core"
#include "absl/types/optional.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

// [Innovation 1] Thread-safe snapshot data for directional 2D degeneracy.
// [Innovation 1] The scan matcher writes this from the local SLAM sensor
// [Innovation 1] thread; cartographer_ros reads a copy for diagnostics.
struct DirectionalDegeneracyMetric {
  bool enabled = false;
  common::Time time = common::FromUniversal(0);
  double confidence = 0.;
  double condition_number = 1.;
  double real_time_correlative_score = 0.;
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
  double longitudinal_scale = 1.;
  double lateral_scale = 1.;
};

// [Innovation 1] Returns the latest metric copy protected by an internal mutex.
DirectionalDegeneracyMetric GetLatestDirectionalDegeneracyMetric();

// [Innovation 1] Records a time-stamped front-end metric in a bounded,
// [Innovation 1] thread-safe history buffer. This is used by Innovation 2 so
// [Innovation 1] backend odometry edges query the metric near their own node
// [Innovation 1] time instead of using a stale "latest" value.
void RecordDirectionalDegeneracyMetric(DirectionalDegeneracyMetric metric,
                                       common::Time time);

// [Innovation 1] Returns the nearest metric copy if its timestamp is within
// [Innovation 1] max_delta. The function never exposes internal iterators or
// [Innovation 1] references, avoiding front-end/backend data races.
absl::optional<DirectionalDegeneracyMetric> QueryDirectionalDegeneracyMetric(
    common::Time time, common::Duration max_delta);

}  // namespace mapping
}  // namespace cartographer

#endif  // [Innovation 1] CARTOGRAPHER_MAPPING_DIRECTIONAL_DEGENERACY_METRIC_H_
