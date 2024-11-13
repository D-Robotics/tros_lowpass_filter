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

#ifndef INCLUDE_LOWPASSFILTERMETHOD_LOWPASSFILTERMETHOD_H_
#define INCLUDE_LOWPASSFILTERMETHOD_LOWPASSFILTERMETHOD_H_

#include <map>
#include <memory>
#include <string>
#include <vector>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "ai_msgs/msg/perception_targets.hpp"

#include "LowPassFilter.h"
#include "OneEuroFilter.h"

namespace tros {

struct FilterParam {
  FilterParam() {}
  FilterParam(double freq, double mincutoff, double beta, double dcutoff) :
    freq_(freq), mincutoff_(mincutoff), beta_(beta), dcutoff_(dcutoff) {
  }

  /*
    "desp" : 
      'beta' is to reduce the delay to target, larger beta, faster tracking. 
      'mincutoff' is to reduce the vibration of the bbox and lmk point, less mincutoff, less vibration
  */ 
  double freq_ = 120;
  double mincutoff_ = 0.01;
  double beta_ = 0.1;
  double dcutoff_ = 1.0;
};

struct pair_filter {
  pair_filter(double freq, double mincutoff = 1.0, double beta_ = 1.0, double dcutoff = 0.3) {
    filter_x = std::make_shared<OneEuroFilter>(freq, mincutoff, beta_, dcutoff);
    filter_y = std::make_shared<OneEuroFilter>(freq, mincutoff, beta_, dcutoff);
  }
  pair_filter(FilterParam filter_param) {
    filter_x = std::make_shared<OneEuroFilter>(
      filter_param.freq_, filter_param.mincutoff_, filter_param.beta_, filter_param.dcutoff_);
    filter_y = std::make_shared<OneEuroFilter>(
      filter_param.freq_, filter_param.mincutoff_, filter_param.beta_, filter_param.dcutoff_);
  }
  std::shared_ptr<OneEuroFilter> filter_x;
  std::shared_ptr<OneEuroFilter> filter_y;
};

struct filtersVector {
  std::vector<std::shared_ptr<pair_filter>> filter_vector;
};

class LowPassFilterNode : public rclcpp::Node {
 public:
  LowPassFilterNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions(),
    std::string node_name = "tros_lowpass_filter");
  ~LowPassFilterNode();

 private:
  // key is type_id
  std::map<std::string, filtersVector> kps_filter_pairs_;
  std::map<std::string, filtersVector> box_filter_pairs_;
  const size_t cache_warnning_size_ = 100;
  uint64_t track_timeout_sec_ = 10;
  // key is type_id, value is the lastest timestamp of second
  std::unordered_map<std::string, uint64_t> track_update_map_;

  double freq_ = 120.0;
  // std::map<std::string, std::shared_ptr<pair_filter>> map_filters_;
  std::map<std::string, FilterParam> map_filter_params_;
  FilterParam filter_default_param_ = FilterParam(120.0, 0.01, 0.1, 1.0);

  std::string config_file_ = "";

  std::string perc_sub_topic_ = "tros_perc";
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr perc_sub_ = nullptr; 
  std::string perc_pub_topic_ = "tros_perc_lowpass_filtered";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr perc_pub_ = nullptr;

  int ParseFromFile(std::string config_file);

  void perception_callback(ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  ai_msgs::msg::PerceptionTargets::SharedPtr DoProcess(const ai_msgs::msg::PerceptionTargets::SharedPtr msg);

  int ClearCache(const ai_msgs::msg::PerceptionTargets::SharedPtr in_msg);

};

}  // namespace tros

#endif  // INCLUDE_MERGEMETHOD_MERGEMETHOD_H_
