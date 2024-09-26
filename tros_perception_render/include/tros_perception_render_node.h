#include <memory>
#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>  
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/sync_policies/exact_time.h"
#include "ai_msgs/msg/perception_targets.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "common.hpp"

namespace tros {

static uint8_t bgr_putpalette[] = {
    0,   0 ,  0  , 244, 35 , 232, 70 , 70 , 70 , 102, 102, 156, 190, 153, 153, 
    153, 153, 153, 250, 170, 30 , 220, 220, 0  , 107, 142, 35 , 152, 251, 152, 
    0  , 130, 180, 220, 20 , 60 , 255, 0  , 0  , 0  , 0  , 142, 0  , 0  , 70 , 
    0  , 60 , 100, 0  , 80 , 100, 0  , 0  , 230, 119, 11 , 32 , 216, 191, 69 , 
    50 , 33 , 199, 108, 59 , 247, 249, 96 , 97 , 97 , 234, 195, 239, 202, 156, 
    81 , 177, 90 , 180, 100, 245, 251, 146, 184, 245, 26 , 209, 56 , 20 , 144, 
    210, 56 , 241, 19 , 75 , 171, 144, 17 , 198, 216, 105, 125, 108, 212, 181, 
    75 , 189, 225, 137, 152, 226, 210, 107, 81 , 130, 189, 63 , 4  , 31 , 139, 
    106, 202, 255, 184, 64 , 56 , 200, 69 , 31 , 62 , 129, 13 , 19 , 235, 0  , 
    255, 129, 8  , 238, 24 , 80 , 176, 115, 54 , 232, 100, 164, 13 , 192, 234, 
    48 , 140, 176, 178, 145, 83 , 115, 225, 250, 18 , 6  , 98 , 34 , 156, 78 , 
    74 , 120, 22 , 185, 5  , 159, 111, 133, 243, 170, 252, 118, 23 , 29 , 143, 
    237, 6  , 163, 104, 231, 87 , 18 , 15 , 185, 45 , 152, 178, 147, 116, 56 , 
    28 , 197, 148, 134, 46 , 205, 243, 200, 47 , 5  , 233, 70 , 224, 88 , 0  , 
    237, 82 , 6  , 180, 104, 75 , 80 , 91 , 20 , 95 , 225, 61 , 91 , 37 , 187, 
    129, 183, 114, 246, 21 , 181, 26 , 90 , 201, 218, 8  , 81 , 97 , 14 , 208, 
    51 , 172, 247
};

static std::vector<cv::Scalar> colors{
    cv::Scalar(255, 0, 0),    // red
    cv::Scalar(255, 255, 0),  // yellow
    cv::Scalar(0, 255, 0),    // green
    cv::Scalar(0, 0, 255),    // blue
};

struct PoseAttribute {
  // 单位米，并使用NaN初始化
  float width = 0.0f / 0.0f;
  float x = 0.0f / 0.0f;
  float y = 0.0f / 0.0f;
  float z = 0.0f / 0.0f;
};

// 系统状态
struct SystemStatus {
  // -1或者空 表示未获取到
  float cpu_usage = -1;
  float mem_usage = -1;
  // 芯片温度
  std::string temperature = "";
};

class TrosPerceptionRenderNode : public rclcpp::Node {
 public:
   TrosPerceptionRenderNode(const rclcpp::NodeOptions &options);
   ~TrosPerceptionRenderNode() = default;

 private:
  rclcpp::TimerBase::SharedPtr timer_;

  using CustomSyncPolicyType = message_filters::sync_policies::ExactTime<ai_msgs::msg::PerceptionTargets,
  sensor_msgs::msg::CompressedImage>;
  using SynchronizerType = message_filters::Synchronizer<CustomSyncPolicyType>;
  std::shared_ptr<SynchronizerType> synchronizer_;

  std::string perception_topic_name_ = "tros_dnn_detection";
  std::string img_topic_name_ = "tros_img";
  std::string pub_render_topic_name_ = "tros_render_img";
   
  message_filters::Subscriber<ai_msgs::msg::PerceptionTargets> sub_perc_;
  message_filters::Subscriber<sensor_msgs::msg::CompressedImage> sub_img_;

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr render_img_publisher_ = nullptr;

  // 查询系统状态的现成
  std::thread system_status_thread_;
  std::shared_ptr<SystemStatus> system_status_ = std::make_shared<SystemStatus>();

  Tools tools_;

  void TopicSyncCallback(
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg1,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg2);

  cv::Mat compressedImageToMat(const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_ptr);

  int Render(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg_perc,
    const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_img,
    cv::Mat &mat);

  // 获取系统状态
  void GetSystemStatus();
};

}
