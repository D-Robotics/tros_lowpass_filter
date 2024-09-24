#include "tros_perception_render_node.h"

namespace tros {

TrosPerceptionRenderNode::TrosPerceptionRenderNode(const rclcpp::NodeOptions &options) :
  Node("tros_perception_render", options) {
  RCLCPP_INFO(this->get_logger(), "TrosPerceptionRenderNode is initializing...");
  perception_topic_name_ = this->declare_parameter("perception_topic_name", perception_topic_name_);
  img_topic_name_ = this->declare_parameter("img_topic_name", img_topic_name_);
  pub_render_topic_name_= this->declare_parameter("pub_render_topic_name", pub_render_topic_name_);

  RCLCPP_WARN_STREAM(this->get_logger(),
    "\n perception_topic_name [" << perception_topic_name_ << "]"
    << "\n img_topic_name [" << img_topic_name_ << "]"
    << "\n pub_render_topic_name [" << pub_render_topic_name_ << "]"
  );

  render_img_publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
    pub_render_topic_name_,
    rclcpp::QoS(10));

  sub_perc_.subscribe(this, perception_topic_name_);
  sub_img_.subscribe(this, img_topic_name_);

  synchronizer_ = std::make_shared<SynchronizerType>(CustomSyncPolicyType(10), sub_perc_, sub_img_);
  synchronizer_->registerCallback(std::bind(&TrosPerceptionRenderNode::TopicSyncCallback,
    this, std::placeholders::_1, std::placeholders::_2));
}

void TrosPerceptionRenderNode::TopicSyncCallback(
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg_perc,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_img) {
  cv::Mat mat;
  Render(msg_perc, msg_img, mat);

  std::vector<uchar> buf;  
  cv::imencode(".jpeg", mat, buf);
  auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();  
  msg->header = msg_img->header;
  msg->format = msg_img->format;  
  msg->data.assign(buf.begin(), buf.end());  
  render_img_publisher_->publish(std::move(*msg));
}  
  
cv::Mat TrosPerceptionRenderNode::compressedImageToMat(
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr img_ptr) {  
    try {  
        // 确保图像数据不是空的  
        if (!img_ptr || img_ptr->data.empty()) {  
            std::cerr << "Compressed image is empty!" << std::endl;  
            return cv::Mat();  
        }  
  
        // 根据压缩格式创建解码器  
        std::vector<uchar> buf(img_ptr->data.begin(), img_ptr->data.end());  
        cv::Mat img_decoded;  
  
        // 假设数据是JPEG格式的，这是最常见的格式  
        if (img_ptr->format == "jpeg") {  
            cv::imdecode(buf, cv::IMREAD_COLOR, &img_decoded);  
        } else if (img_ptr->format == "png") {  
            cv::imdecode(buf, cv::IMREAD_UNCHANGED, &img_decoded);  
        } else {  
            std::cerr << "Unsupported image format: " << img_ptr->format << std::endl;  
            return cv::Mat();  
        }  
  
        // 如果解码失败，返回空的Mat  
        if (img_decoded.empty()) {  
            std::cerr << "Failed to decode image!" << std::endl;  
            return cv::Mat();  
        }  
  
        return img_decoded;  
    } catch (const std::exception& e) {  
        std::cerr << "Error processing image: " << e.what() << std::endl;  
        return cv::Mat();  
    }  
}

int TrosPerceptionRenderNode::Render(
  const ai_msgs::msg::PerceptionTargets::ConstSharedPtr msg_perc,
  const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg_img,
  cv::Mat &mat) {
  if (!msg_perc || !msg_img) return -1;
  mat = compressedImageToMat(msg_img);  
  if (mat.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("ImageUtils"), "Failed to decode image.");
    return -1;
  }  

  RCLCPP_INFO(rclcpp::get_logger("ImageUtils"),
              "target size: %d",
              msg_perc->targets.size());
  for (size_t idx = 0; idx < msg_perc->targets.size(); idx++) {
    const auto &target = msg_perc->targets.at(idx);
    RCLCPP_INFO(rclcpp::get_logger("ImageUtils"),
                "target type: %s, rois.size: %d",
                target.type.c_str(),
                target.rois.size());
    auto &color = colors[idx % colors.size()];
    for (const auto &roi : target.rois) {
      RCLCPP_INFO(
          rclcpp::get_logger("ImageUtils"),
          "roi.type: %s, x_offset: %d y_offset: %d width: %d height: %d",
          roi.type.c_str(),
          roi.rect.x_offset,
          roi.rect.y_offset,
          roi.rect.width,
          roi.rect.height);
      cv::rectangle(mat,
                    cv::Point(roi.rect.x_offset, roi.rect.y_offset),
                    cv::Point(roi.rect.x_offset + roi.rect.width,
                              roi.rect.y_offset + roi.rect.height),
                    color,
                    3);
      std::string roi_type = target.type;
      if (!roi.type.empty()) {
        roi_type = roi.type;
      }
      if (!roi_type.empty()) {
        cv::putText(mat,
                    roi_type,
                    cv::Point2f(roi.rect.x_offset, roi.rect.y_offset - 10),
                    cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1.5);
      }
    }

    if (!target.rois.empty()) {
      auto floatToString = [](float value) -> std::string {  
        std::ostringstream oss;  
        oss << std::fixed << std::setprecision(2) << value;  
        return oss.str();  
      };  

      int pt_x = target.rois.front().rect.x_offset;
      int pt_y = target.rois.front().rect.y_offset;
      int y_offset = 0;
      auto &color_attr = colors.at(std::min(2, static_cast<int>(colors.size() - 1)));
      // 过滤需要可视化的属性
      PoseAttribute pose_attr;
      for (const auto& attr : target.attributes) {
        if (attr.type == "width_cm") {
          pose_attr.width = attr.value / 100.0;
        } else if (attr.type == "x_cm") {
          pose_attr.x = attr.value / 100.0;
        } else if (attr.type == "y_cm") {
          pose_attr.y = attr.value / 100.0;
        } else if (attr.type == "z_cm") {
          pose_attr.z = attr.value / 100.0;
        }
      }

      if (!std::isnan(pose_attr.x) && !std::isnan(pose_attr.y) && !std::isnan(pose_attr.z)) {
        y_offset += 30;
        // int render_y = pt_y + y_offset;
        // if (render_y < 0 ) render_y = 0;
        // if (render_y > mat.rows) render_y = mat.rows;
        cv::putText(mat,
                    "xyz (" + floatToString(pose_attr.x) + ", " + floatToString(pose_attr.y) + ", " + floatToString(pose_attr.z) + ")",
                    cv::Point2f(pt_x, pt_y + y_offset),
                    cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    color_attr,
                    2.0);
      }
      if (!std::isnan(pose_attr.width)) {
        y_offset += 30;
        cv::putText(mat,
                    "width (" + floatToString(pose_attr.width) + ")",
                    cv::Point2f(pt_x, pt_y + y_offset),
                    cv::HersheyFonts::FONT_HERSHEY_SIMPLEX,
                    1.0,
                    color_attr,
                    2.0);

      }
    }

    for (const auto &lmk : target.points) {
      for (const auto &pt : lmk.point) {
        cv::circle(mat, cv::Point(pt.x, pt.y), 3, color, 3);
      }
    }

    if (!target.captures.empty()) {
      float alpha_f = 0.5;
      for (auto capture: target.captures) {
        int parsing_width = capture.img.width;
        int parsing_height = capture.img.height;
        cv::Mat parsing_img(parsing_height, parsing_width, CV_8UC3);
        uint8_t *parsing_img_ptr = parsing_img.ptr<uint8_t>();

        for (int h = 0; h < parsing_height; ++h) {
          for (int w = 0; w < parsing_width; ++w) {
            auto id = static_cast<size_t>(capture.features[h * parsing_width + w]);
            id = id >= 80? (id % 80) + 1: id;
            *parsing_img_ptr++ = bgr_putpalette[id * 3];
            *parsing_img_ptr++ = bgr_putpalette[id * 3 + 1];
            *parsing_img_ptr++ = bgr_putpalette[id * 3 + 2];
          }
        }

        cv::resize(parsing_img, parsing_img, mat.size(), 0, 0);
        // alpha blending
        cv::Mat dst;
        addWeighted(mat, alpha_f, parsing_img, 1 - alpha_f, 0.0, dst);
        mat = std::move(dst);
      }
    }
  }

  // std::string saving_path = "render_" + msg_perc->header.frame_id + "_" +
  //                           std::to_string(msg_perc->header.stamp.sec) + "_" +
  //                           std::to_string(msg_perc->header.stamp.nanosec) +
  //                           ".jpeg";
  // RCLCPP_WARN(rclcpp::get_logger("ImageUtils"),
  //             "Draw result to file: %s",
  //             saving_path.c_str());
  // cv::imwrite(saving_path, mat);
  return 0;
}


}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(tros::TrosPerceptionRenderNode)