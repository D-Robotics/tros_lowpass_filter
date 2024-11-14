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

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "ai_msgs/msg/perception_targets.hpp"
#include "tros_ai_fusion_msgs/srv/topic_manage.hpp"

namespace tros {

class TrosAiMsgFusionNode : public rclcpp::Node {
 public:
   TrosAiMsgFusionNode(const rclcpp::NodeOptions &options);
   ~TrosAiMsgFusionNode() = default;

 private:
   rclcpp::TimerBase::SharedPtr timer_;

   using CustomSyncPolicyType = message_filters::sync_policies::ExactTime<ai_msgs::msg::PerceptionTargets, ai_msgs::msg::PerceptionTargets>;
   using SynchronizerType = message_filters::Synchronizer<CustomSyncPolicyType>;
   // key is topic name
   std::map<std::string, std::shared_ptr<SynchronizerType>> synchronizers_map_;
   // key is topic name
   std::map<std::string, message_filters::Subscriber<ai_msgs::msg::PerceptionTargets>> subs_map_;
   std::string fusion_topic_name_base_ = "hobot_dnn_detection";
   message_filters::Subscriber<ai_msgs::msg::PerceptionTargets> base_sub_;

   // 不包含 base topic
   std::vector<std::string> fusion_topic_names_ = {};

  std::string srv_topic_manage_topic_name_ = "tros_topic_manage";
   rclcpp::Service<tros_ai_fusion_msgs::srv::TopicManage>::SharedPtr srv_topic_manage_ = nullptr;

  // key is topic name
   using MsgCacheType = std::map<std::string, ai_msgs::msg::PerceptionTargets::SharedPtr>;
  // key is time stamp
   std::map<std::string, MsgCacheType> msg_cache_;
   std::mutex sync_msgs_cache_mutex_;
   size_t sync_msgs_cache_max_size_ = 10;

  std::shared_ptr<std::chrono::high_resolution_clock::time_point> output_tp_ =
      nullptr;
  int output_frameCount_ = 0;
  int smart_fps_ = -1;
  std::mutex frame_stat_mtx_;

  std::string pub_fusion_topic_name_ = "fusion_ai_msg";
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr ai_msg_publisher_ = nullptr;

   void topic_manage_callback(
   const std::shared_ptr<rmw_request_id_t>/*request_header*/,
   const std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage::Request>/*request*/,
   const std::shared_ptr<tros_ai_fusion_msgs::srv::TopicManage::Response>/*response*/);
   void TopicSyncCallback(
   std::string base_topic, std::string fusion_topic,
   const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg1,
   const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg2);
   void callback_base_sub(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg);

   void FusionMsg(MsgCacheType msg_cache);
   std::vector<std::string> RegisterSynchronizer(
    const std::vector<std::string>& topic_names);

  // 融合方法，false: 直接拷贝所有成员；true: 根据内容融合，过滤相同的roi, kps
  bool enable_filter_ = true;
};

}
