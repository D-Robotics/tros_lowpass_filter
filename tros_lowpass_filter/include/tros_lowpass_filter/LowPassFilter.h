// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDE_LOWPASSFILTER_H_
#define INCLUDE_LOWPASSFILTER_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <ctime>

namespace tros {
// -----------------------------------------------------------------
typedef double TimeStamp;  // in seconds

static const TimeStamp UndefinedTime = -1.0;
// -----------------------------------------------------------------
class LowPassFilter {
  double y, a, s;
  bool initialized;

  void setAlpha(double alpha) {
    if (alpha <= 0.0 || alpha > 1.0) {
      throw std::range_error("alpha should be in (0.0., 1.0]");
    }
    a = alpha;
  }

 public:
  explicit LowPassFilter(double alpha, double initval = 0.0) {
    y = s = initval;
    setAlpha(alpha);
    initialized = false;
  }

  double filter(double value) {
    double result;
    if (initialized) {
      result = a * value + (1.0 - a) * s;
    } else {
      result = value;
      initialized = true;
    }
    y = value;
    s = result;
    return result;
  }

  double filterWithAlpha(double value, double alpha) {
    setAlpha(alpha);
    return filter(value);
  }

  bool hasLastRawValue(void) {
    return initialized;
  }

  double lastRawValue(void) {
    return y;
  }
};
}  // namespace tros

#endif  // INCLUDE_LOWPASSFILTER_H_
