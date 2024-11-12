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

#include "tros_lowpass_filter/LowPassFilterNode.h"
#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <utility>

namespace tros {

LowPassFilterNode::LowPassFilterNode(const rclcpp::NodeOptions & node_options,
  std::string node_name) : rclcpp::Node(node_name, node_options) {
  perc_sub_topic_ = this->declare_parameter("perc_sub_topic", perc_sub_topic_);
  perc_pub_topic_ = this->declare_parameter("perc_pub_topic", perc_pub_topic_);

  RCLCPP_WARN_STREAM(this->get_logger(),
    "\n perc_sub_topic: " << perc_sub_topic_
    << "\n perc_pub_topic: " << perc_pub_topic_);

  {
    // face/face_kps
    std::string key = "face";
    double dcutoff = 1.0;
    double beta = 0.02;
    double mincutoff = 0.01;
    double freq = 120;
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
    key = "face_kps";
    beta = 0.1;
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
  }
  {
    // hand/hand_kps
    std::string key = "hand";
    double dcutoff = 1.0;
    double beta = 0.1;
    double mincutoff = 0.01;
    double freq = 120;
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
    key = "hand_kps";
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
  }
  {
    // head
    std::string key = "head";
    double dcutoff = 1.0;
    double beta= 0.1;
    double mincutoff = 0.01;
    double freq = 120;
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
  }
  {
    // body/body_kps
    std::string key = "body";
    double dcutoff = 1.0;
    double beta= 0.02;
    double mincutoff = 0.01;
    double freq = 120;
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
    key = "body_kps";
    map_filter_params_[key] = std::make_shared<FilterParam>(freq, mincutoff, beta, dcutoff);
  }

  perc_sub_ = this->create_subscription<ai_msgs::msg::PerceptionTargets>(
    perc_sub_topic_, rclcpp::QoS(10),
    std::bind(&LowPassFilterNode::perception_callback, this, std::placeholders::_1));

  perc_pub_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
    perc_pub_topic_, rclcpp::QoS(10));
}

LowPassFilterNode::~LowPassFilterNode() {
}

void LowPassFilterNode::perception_callback(
  ai_msgs::msg::PerceptionTargets::SharedPtr msg) {
  ai_msgs::msg::PerceptionTargets::SharedPtr new_msg = DoProcess(msg);
  RCLCPP_DEBUG(this->get_logger(),
    "in targets size: %ld, out targets size: %ld",
    msg->targets.size(), new_msg->targets.size());
  if (new_msg && perc_pub_) {
    perc_pub_->publish(std::move(*new_msg));
  }
}

ai_msgs::msg::PerceptionTargets::SharedPtr LowPassFilterNode::DoProcess(
  const ai_msgs::msg::PerceptionTargets::SharedPtr in_msg) {
  if (!in_msg) return nullptr;

  ai_msgs::msg::PerceptionTargets::SharedPtr msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();
  msg->set__header(in_msg->header);
  msg->set__fps(in_msg->fps);
  msg->set__perfs(in_msg->perfs);
  msg->set__disappeared_targets(in_msg->disappeared_targets);

  static double timestamp = 0.0;

  auto find_filter = [this](std::string type)->std::shared_ptr<pair_filter>{
    if (map_filter_params_.find(type) != map_filter_params_.end()) {
      return std::make_shared<pair_filter>(*map_filter_params_[type]);
    } else {
      return std::make_shared<pair_filter>(filter_default_param_);
    }
  };

  for (auto & target : in_msg->targets) {
    ai_msgs::msg::Target new_target;
    new_target.set__type(target.type);
    new_target.set__track_id(target.track_id);
    new_target.set__attributes(target.attributes);
    new_target.set__captures(target.captures);

    std::string target_key = target.type + std::to_string(target.track_id);

    for (const auto& roi : target.rois) {
      std::string key = target_key + roi.type;
      if (box_filter_pairs_.find(key) == box_filter_pairs_.end()) {
        filtersVector box_filterVector;
        for (size_t j = 0; j < 2; j++) {
          box_filterVector.filter_vector.push_back(find_filter(roi.type));
        }
        box_filter_pairs_.insert(
            std::pair<std::string, filtersVector>(key, box_filterVector));
        new_target.rois.push_back(roi);
        track_update_map_[key] =
          std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        continue;
      } else {
        ai_msgs::msg::Roi new_roi;
        new_roi.set__type(roi.type);
        new_roi.set__confidence(roi.confidence);

        auto x1 = roi.rect.x_offset;
        auto y1 = roi.rect.y_offset;
        auto x2 = roi.rect.x_offset + roi.rect.width;
        auto y2 = roi.rect.y_offset + roi.rect.height;
        if (box_filter_pairs_[key].filter_vector.size() != 2) {
          new_roi.set__rect(roi.rect);
          continue;
        } else {
          auto new_x1 = box_filter_pairs_[key].filter_vector[0]->
            filter_x->filter(x1, timestamp);
          auto new_y1 = box_filter_pairs_[key].filter_vector[0]->
            filter_y->filter(y1, timestamp);
          auto new_x2 = box_filter_pairs_[key].filter_vector[1]->
            filter_x->filter(x2, timestamp);
          auto new_y2 = box_filter_pairs_[key].filter_vector[1]->
            filter_y->filter(y2, timestamp);

          sensor_msgs::msg::RegionOfInterest new_rect;
          new_rect.x_offset = new_x1;
          new_rect.y_offset = new_y1;
          new_rect.width = new_x2 - new_x1;
          new_rect.height = new_y2 - new_y1;
          new_roi.set__rect(new_rect);

          track_update_map_[key] =
            std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        }
        new_target.rois.push_back(new_roi);
      }
    }

    for (const auto& point : target.points) {
      std::string key = target_key + point.type;
      if (kps_filter_pairs_.find(key) == kps_filter_pairs_.end()) {
        filtersVector kps_filterVector;
        for (size_t j = 0; j < point.point.size(); j++) {
          // auto filter = std::make_shared<pair_filter>(freq_, mincutoff_, beta_, dcutoff_);
          kps_filterVector.filter_vector.push_back(find_filter(point.type));
        }
        kps_filter_pairs_.insert(
            std::pair<std::string, filtersVector>(key, kps_filterVector));
        new_target.points.push_back(point);
        track_update_map_[key] =
          std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch()).count();
        continue;
      } else {
        if (point.point.size() !=
            kps_filter_pairs_[key].filter_vector.size()) {
          new_target.points.push_back(point);
          continue;
        } else {
          ai_msgs::msg::Point new_point;
          new_point.set__type(point.type);
          new_point.set__confidence(point.confidence);
          for (size_t j = 0; j < point.point.size(); j++) {
            geometry_msgs::msg::Point32 new_pt;
            new_pt.x = kps_filter_pairs_[key].filter_vector[j]->
              filter_x->filter(point.point[j].x, timestamp);
            new_pt.y = kps_filter_pairs_[key].filter_vector[j]->
              filter_y->filter(point.point[j].y, timestamp);
            new_point.point.push_back(new_pt);
          }
          new_target.points.push_back(new_point);
          track_update_map_[key] =
            std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        }
      }
    }

    msg->targets.push_back(new_target);
  }

  timestamp += 1.0/freq_;

  ClearCache(in_msg);

  if (kps_filter_pairs_.size() >= cache_warnning_size_ || box_filter_pairs_.size() >= cache_warnning_size_) {
    RCLCPP_WARN(this->get_logger(),
      "cache size is %ld", cache_warnning_size_);
  }

  return msg;
}


int LowPassFilterNode::ClearCache(const ai_msgs::msg::PerceptionTargets::SharedPtr in_msg) {
  for (auto & target : in_msg->disappeared_targets) {
    std::string target_key = target.type + std::to_string(target.track_id);
    for (const auto& roi : target.rois) {
      std::string key = target_key + roi.type;
      if (box_filter_pairs_.find(key) != box_filter_pairs_.end()) {
        box_filter_pairs_.erase(key);
      }
    }
    for (const auto& point : target.points) {
      std::string key = target_key + point.type;
      if (kps_filter_pairs_.find(key) != kps_filter_pairs_.end()) {
        kps_filter_pairs_.erase(key);
      }
    }
  }
  
  if (!track_update_map_.empty()) {
    // clear timeout track cache
    auto time_now = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    for (auto iter = track_update_map_.begin(); iter != track_update_map_.end(); iter++) {
      auto track_update_last = iter->second;
      auto key = iter->first;
      if (time_now - track_update_last > track_timeout_sec_) {
        if (box_filter_pairs_.find(key) != box_filter_pairs_.end()) {
          box_filter_pairs_.erase(key);
        }
        if (kps_filter_pairs_.find(key) != kps_filter_pairs_.end()) {
          kps_filter_pairs_.erase(key);
        }
        iter = track_update_map_.erase(iter);
        if (iter == track_update_map_.end()) {
          break;
        }
      }
    }
  }
  
  return 0;
}


}  // namespace tros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tros::LowPassFilterNode)