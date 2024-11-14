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

#include "tros_ai_fusion_node.h"

namespace tros {

TrosAiMsgFusionNode::TrosAiMsgFusionNode(const rclcpp::NodeOptions &options) :
  Node("tros_ai_fusion", options) {
  RCLCPP_INFO(this->get_logger(), "TrosAiMsgFusionNode is initializing...");
  this->declare_parameter("topic_names_fusion", rclcpp::ParameterValue(fusion_topic_names_));
  this->get_parameter("topic_names_fusion", fusion_topic_names_);
  fusion_topic_name_base_ = this->declare_parameter("topic_name_base", fusion_topic_name_base_);
  pub_fusion_topic_name_= this->declare_parameter("pub_fusion_topic_name", pub_fusion_topic_name_);
  enable_filter_ = this->declare_parameter("enable_filter", enable_filter_);

  std::stringstream ss;
  for (const auto &topic_name : fusion_topic_names_) {
    ss << "\n\t" << topic_name;
  }
  RCLCPP_WARN_STREAM(this->get_logger(),
    "\n topic_name_base [" << fusion_topic_name_base_ << "]"
    << "\n topic_names_fusion: " << ss.str()
    << "\n pub_fusion_topic_name [" << pub_fusion_topic_name_ << "]"
    << "\n enable_filter [" << enable_filter_ << "]"
    << "\n srv_topic_manage_topic_name [" << srv_topic_manage_topic_name_
      << "], you can do action [" << tros_ai_fusion_msgs::srv::TopicManage::Request::ADD
      << "|" << tros_ai_fusion_msgs::srv::TopicManage::Request::DELETE
      << "|" << tros_ai_fusion_msgs::srv::TopicManage::Request::GET << "]"
  );

  ai_msg_publisher_ = this->create_publisher<ai_msgs::msg::PerceptionTargets>(
    pub_fusion_topic_name_,
    rclcpp::QoS(10));
  base_sub_.subscribe(this, fusion_topic_name_base_);
  base_sub_.registerCallback(
    std::bind(
      &TrosAiMsgFusionNode::callback_base_sub,
      this,
      std::placeholders::_1));
  srv_topic_manage_ = this->create_service<tros_ai_fusion_msgs::srv::TopicManage>(
    srv_topic_manage_topic_name_,
    std::bind(&TrosAiMsgFusionNode::topic_manage_callback,
    this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
  RegisterSynchronizer(fusion_topic_names_);
}

void TrosAiMsgFusionNode::topic_manage_callback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage::Request> request,
  const std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage::Response> response) {
  RCLCPP_WARN(get_logger(), "topic_manage_callback request->action [%s]",
    request->action.data());

  if (request->action == tros_ai_fusion_msgs::srv::TopicManage::Request::ADD) {
    if (request->topics.empty()) {
      response->result = false;
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "topic name is empty");
      return;
    }
    
    std::stringstream ss;
    ss << "Add fusion topics: ";
    for (const auto& name : request->topics) {
      ss << name << ", ";
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
      ss.str()
      );

    response->result = true;
    auto registered_topics = RegisterSynchronizer(request->topics);
    if (!registered_topics.empty()) {
      fusion_topic_names_.insert(fusion_topic_names_.end(),
        registered_topics.begin(), registered_topics.end());
    }
  } else if (request->action == tros_ai_fusion_msgs::srv::TopicManage::Request::DELETE)
  {
    if (request->topics.empty()) {
      response->result = false;
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "topic name is empty");
      return;
    }
    
    std::stringstream ss;
    ss << "Delete fusion topics: ";
    for (const auto& name : request->topics) {
      ss << name << ", ";
    }
    RCLCPP_WARN_STREAM(this->get_logger(),
      ss.str()
      );

    response->result = true;

    for (const auto& name : request->topics) {
      std::vector<std::string>::iterator iter;
      for (iter = fusion_topic_names_.begin(); iter <= fusion_topic_names_.end(); iter++) {
        if (iter == fusion_topic_names_.end()) {
          RCLCPP_ERROR(this->get_logger(),
            "Delete fusion topic [%s] failed, which is not existed",
            name.data());
          break;
        }
        if (*iter == name) {
          break;
        }
      }
      if (iter == fusion_topic_names_.end()) {
        break;
      }

      if (synchronizers_map_.find(name) == synchronizers_map_.end() ||
        subs_map_.find(name) == subs_map_.end()
        ) {
        RCLCPP_ERROR(this->get_logger(),
          "Delete fusion topic [%s] failed, which is not existed",
          name.data());
        continue;
      }

      fusion_topic_names_.erase(iter);
      synchronizers_map_.erase(name);
      subs_map_.erase(name);
      RCLCPP_WARN_STREAM(this->get_logger(),
        "Delete fusion topic: " << name << " success"
        );
    }
    
  } else if (request->action == tros_ai_fusion_msgs::srv::TopicManage::Request::GET)
  {
    response->result = true;
    response->topics = fusion_topic_names_;    
  } else {
    RCLCPP_ERROR(this->get_logger(),
      "action [%s] is invalid", request->action.c_str());
    response->result = false;
    return;
  }

  return;
}

void TrosAiMsgFusionNode::TopicSyncCallback(
  std::string base_topic, std::string fusion_topic,
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg1,
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg2) {
  auto copy_msg = [this](const ai_msgs::msg::PerceptionTargets::ConstSharedPtr in_msg)
    ->ai_msgs::msg::PerceptionTargets::SharedPtr{
    auto msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();
    msg->set__header(in_msg->header);
    msg->set__fps(in_msg->fps);
    msg->set__perfs(in_msg->perfs);
    msg->set__targets(in_msg->targets);
    msg->set__disappeared_targets(in_msg->disappeared_targets);

    return msg;
  };

  std::string time = std::to_string(msg1->header.stamp.sec) + "." + std::to_string(msg1->header.stamp.nanosec);

  RCLCPP_INFO_THROTTLE(this->get_logger(),
    *this->get_clock(), 1000,
    "TopicSyncCallback at [%s] with topics [%s] [%s]",
    time.data(), base_topic.data(), fusion_topic.data());

  std::unique_lock<std::mutex> lock(sync_msgs_cache_mutex_);
  auto iter = msg_cache_.find(time);
  if (iter == msg_cache_.end()) {
    RCLCPP_INFO(
      this->get_logger(),
      "Insert msgs at [%s].",
      time.data());
    MsgCacheType msg_cache;
    msg_cache[base_topic] = copy_msg(msg1);
    msg_cache[fusion_topic] = copy_msg(msg2);
    msg_cache_[time] = std::move(msg_cache);
  } else {
    iter->second[fusion_topic] = copy_msg(msg2);
    RCLCPP_INFO(
      this->get_logger(),
      "Messages at [%s] have [%ld] msgs.",
      time.data(), iter->second.size());
  }
  
  iter = msg_cache_.find(time);
  if (iter->second.size() == fusion_topic_names_.size() + 1) {
    RCLCPP_INFO(
      this->get_logger(),
      "Messages at [%s] are ready with [%ld] msgs.",
      time.data(), iter->second.size());

    FusionMsg(iter->second);
    msg_cache_.erase(iter);
  }

  while (msg_cache_.size() > sync_msgs_cache_max_size_) {
    std::stringstream ss;
    ss << "msg_cache_ size:" << msg_cache_.size()
    << " exceeds limit:" << sync_msgs_cache_max_size_
    << ", erase ts:" << msg_cache_.begin()->first
    << ", which has recved " << msg_cache_.begin()->second.size() << " topics [";
    for (auto itr = msg_cache_.begin()->second.begin(); itr != msg_cache_.begin()->second.end(); itr++) {
      ss << itr->first << " ";
    }
    ss << "], should have " << fusion_topic_names_.size() + 1 << " topics [";  
    for (const auto& name : fusion_topic_names_) {
      ss << name << " ";
    }
    ss << fusion_topic_name_base_ << "]";

    RCLCPP_INFO_STREAM(this->get_logger(),
      ss.str()
      );
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      ss.str()
      );
    
    msg_cache_.erase(msg_cache_.begin());
  }
}


void TrosAiMsgFusionNode::FusionMsg(MsgCacheType msg_cache) {
  assert(ai_msg_publisher_);

  auto pub_ai_msg = std::make_shared<ai_msgs::msg::PerceptionTargets>();
  auto base_msg = msg_cache[fusion_topic_name_base_];

  pub_ai_msg->set__header(base_msg->header);
  pub_ai_msg->set__fps(base_msg->fps);
  pub_ai_msg->set__perfs(base_msg->perfs);

  pub_ai_msg->set__targets(base_msg->targets);
  pub_ai_msg->set__disappeared_targets(base_msg->disappeared_targets);


  RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(),
    *this->get_clock(), 1000,
    "Fusion msg at " << base_msg->header.stamp.sec << "." << base_msg->header.stamp.nanosec
    << " with msg size " << msg_cache.size()
  );

  for (auto iter = msg_cache.begin(); iter != msg_cache.end(); ++iter) {
    if (iter->first == fusion_topic_name_base_) {
      continue;
    }
    for (const auto& target : iter->second->targets) {
      pub_ai_msg->targets.push_back(target);
    }
    for (const auto& target : iter->second->disappeared_targets) {
      pub_ai_msg->disappeared_targets.push_back(target);
    }
  }

  if (enable_filter_) {
    std::set<std::string> keys;
    std::map<std::string, ai_msgs::msg::Target> map_targets;
    for (auto & target : pub_ai_msg->targets) {
      if (target.rois.empty()) continue;

      std::string target_key = target.type + std::to_string(target.track_id) + target.rois.front().type;
      if (map_targets.find(target_key) == map_targets.end()) {
        map_targets[target_key] = target;

        for (const auto& roi : target.rois) {
          // 默认一个target只有一个roi
          std::string roi_key = target_key;
          if (keys.find(roi_key) == keys.end()) {
            keys.insert(roi_key);
          }
        }
        for (const auto& attr : target.attributes) {
          std::string attr_key = target_key + attr.type + std::to_string(attr.value);
          if (keys.find(attr_key) == keys.end()) {
            keys.insert(attr_key);
          }
        }
        for (const auto& point : target.points) {
          std::string kps_key = target_key + point.type;
          if (keys.find(kps_key) == keys.end()) {
            keys.insert(kps_key);
          }
        }

        continue;
      } else {
        for (const auto& roi : target.rois) {
          std::string roi_key = target_key;
          if (keys.find(roi_key) == keys.end()) {
            keys.insert(roi_key);
            map_targets[target_key].rois.push_back(roi);
          } else {
            continue;
          }
        }

        for (const auto& attr : target.attributes) {
          std::string attr_key = target_key + attr.type;
          if (keys.find(attr_key) == keys.end()) {
            keys.insert(attr_key);
            map_targets[target_key].attributes.push_back(attr);
          } else {
            continue;
          }
        }

        for (const auto& point : target.points) {
          std::string kps_key = target_key + point.type;
          if (keys.find(kps_key) == keys.end()) {
            keys.insert(kps_key);
            map_targets[target_key].points.push_back(point);
          } else {
            continue;
          }
        }

        for (const auto& capture : target.captures) {
          map_targets[target_key].captures.push_back(capture);
        }
      }
    }
    pub_ai_msg->targets.clear();
    for (const auto& map_target : map_targets) {
      pub_ai_msg->targets.push_back(map_target.second);
    }
  }
  
  // clear target and roi types
  // for (auto & target : pub_ai_msg->targets) {
  //   if (target.type == "parking_space") {
  //     // web会根据 target type 判断是否是分割
  //     continue;
  //   }
  //   target.type = "";
  //   for (auto & roi : target.rois) {
  //     roi.type = "";
  //   }
  // }
  
  
  {
    std::unique_lock<std::mutex> lk(frame_stat_mtx_);
    if (!output_tp_) {
      output_tp_ =
          std::make_shared<std::chrono::high_resolution_clock::time_point>();
      *output_tp_ = std::chrono::system_clock::now();
    }
    auto tp_now = std::chrono::system_clock::now();
    output_frameCount_++;
    auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                        tp_now - *output_tp_)
                        .count();
    if (interval >= 5000) {
      float out_fps = static_cast<float>(output_frameCount_) /
                      (static_cast<float>(interval) / 1000.0);
      RCLCPP_WARN(this->get_logger(),
                  "Pub topic %s fps %.2f",
                  pub_fusion_topic_name_.data(), out_fps);

      smart_fps_ = round(out_fps);
      output_frameCount_ = 0;
      *output_tp_ = std::chrono::system_clock::now();
    }
  }
  
  ai_msg_publisher_->publish(std::move(*pub_ai_msg));
}

std::vector<std::string> TrosAiMsgFusionNode::RegisterSynchronizer(const std::vector<std::string>& topic_names) {
  std::vector<std::string> registered_topics;
  for (auto & topic : topic_names) {
    if (topic.empty()) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "topic name is empty");
      continue;
    } else {
      if (topic == fusion_topic_name_base_) {
        RCLCPP_ERROR(this->get_logger(),
          "Fusion topic name [%s] is same with fusion_topic_name_base [%s]",
          topic.data(), fusion_topic_name_base_.data());
        rclcpp::shutdown();
        return registered_topics;
      }

      if (synchronizers_map_.find(topic) != synchronizers_map_.end()) {
        RCLCPP_ERROR_STREAM(this->get_logger(),
          "topic name already exists: " << topic);
        continue;
      } else {
        registered_topics.push_back(topic);
        RCLCPP_WARN_STREAM(this->get_logger(),
          "topic: " << topic << ", sync with base topic: " << fusion_topic_name_base_);
        subs_map_[topic].subscribe(this, topic);
        auto sync = std::make_shared<SynchronizerType>(CustomSyncPolicyType(10), base_sub_, subs_map_[topic]);
        synchronizers_map_[topic] = sync;
        synchronizers_map_[topic]->registerCallback(std::bind(&TrosAiMsgFusionNode::TopicSyncCallback,
          this, fusion_topic_name_base_, topic,
          std::placeholders::_1, std::placeholders::_2));
      }
    }
  }
  
  RCLCPP_WARN(get_logger(),
    "Registered [%ld] topics with [%s], synchronizers_map_ size [%ld]",
    registered_topics.size(), fusion_topic_name_base_.data(), synchronizers_map_.size());
  return registered_topics;
}

void TrosAiMsgFusionNode::callback_base_sub(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg) {
  if (!msg) return;
  if (!fusion_topic_names_.empty()) return;

  RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(),
    *this->get_clock(), 1000,
    "Fusion topics are empty, passthrough msg"
  );
  assert(ai_msg_publisher_);
  ai_msg_publisher_->publish(*msg);
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tros::TrosAiMsgFusionNode)