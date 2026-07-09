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

namespace cartographer {
namespace mapping {

// [Innovation 1] Thread-safe snapshot data for directional 2D degeneracy.
// [Innovation 1] The scan matcher writes this from the local SLAM sensor
// [Innovation 1] thread; cartographer_ros reads a copy for diagnostics.
struct DirectionalDegeneracyMetric {
  bool enabled = false;
  double confidence = 0.;
  double condition_number = 1.;
  double real_time_correlative_score = 0.;
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
  double longitudinal_scale = 1.;
  double lateral_scale = 1.;
};

// [Innovation 1] Returns the latest metric copy protected by an internal mutex.
DirectionalDegeneracyMetric GetLatestDirectionalDegeneracyMetric();

}  // [Innovation 1] namespace mapping
}  // [Innovation 1] namespace cartographer

#endif  // [Innovation 1] CARTOGRAPHER_MAPPING_DIRECTIONAL_DEGENERACY_METRIC_H_
