#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "assignment_interfaces/srv/take_photo.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

using TakePhoto = assignment_interfaces::srv::TakePhoto;

class TakePhotoServiceNode : public rclcpp::Node
{
public:
  TakePhotoServiceNode()
  : Node("take_photo_service_node"),
    show_image_(declare_parameter<bool>("show_image", true))
  {
    
    // ROS parameters: defaults = simulation, overridden by config/sim.yaml or config/rosbot.yaml
    // image_topic is the full name of the COMPRESSED (JPEG) camera stream
    const std::string image_topic =
      declare_parameter<std::string>("image_topic", "/camera/image/compressed");
    const std::string marker_image_topic =
      declare_parameter<std::string>("marker_image_topic", "/marker_image");

    service_ = create_service<TakePhoto>(
      "take_photo",
      std::bind(
        &TakePhotoServiceNode::handle_service, this,
        std::placeholders::_1, std::placeholders::_2));

    // marker_image_pub_ = create_publisher<sensor_msgs::msg::Image>(marker_image_topic, 10);
    marker_image_pub_ = image_transport::create_publisher(this, marker_image_topic);

    dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    detector_parameters_ = cv::aruco::DetectorParameters::create();

    if (show_image_) {
      cv::namedWindow("ArUco detection", cv::WINDOW_AUTOSIZE);
    }

    // plain subscription to the CompressedImage topic (same as the Python detector).
    // sensor_data QoS (best effort) matches both the Gazebo bridge and the ROSbot camera
    image_subscriber_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      image_topic, rclcpp::SensorDataQoS(),
      std::bind(
        &TakePhotoServiceNode::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Take-Photo service ready: listening on %s, publishing on %s, show_image=%s",
      image_topic.c_str(), marker_image_topic.c_str(), show_image_ ? "true" : "false");
  }

  ~TakePhotoServiceNode() override
  {
    if (show_image_) {
      cv::destroyWindow("ArUco detection");
    }
  }

private:
  void handle_service(
    const std::shared_ptr<TakePhoto::Request> request,
    std::shared_ptr<TakePhoto::Response> response)
  {
    if (!request->request || marker_image_.empty()) {
        response->marker_id = -1;
        response->success = false;
        return;
    }
    cv::Mat gray_image;
    cv::cvtColor(marker_image_, gray_image, cv::COLOR_BGR2GRAY);

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(
    gray_image, dictionary_, marker_corners, marker_ids, detector_parameters_);

    for (size_t i = 0; i < marker_ids.size(); ++i) {
        const auto & corners = marker_corners[i];
        cv::Point2f center(0.0F, 0.0F);
        for (const auto & corner : corners) {
            center += corner;
        }
        center *= 1.0F / static_cast<float>(corners.size());

        float max_distance = 0.0F;
        for (const auto & corner : corners) {
            const cv::Point2f offset = corner - center;
            max_distance = std::max(max_distance, std::sqrt(offset.dot(offset)));
        }

        const cv::Point circle_center(cvRound(center.x), cvRound(center.y));
        const int radius = static_cast<int>(std::ceil(max_distance)) + 3;
        cv::circle(marker_image_, circle_center, radius, cv::Scalar(0, 255, 0), 2);

        const std::string label = "id " + std::to_string(marker_ids[i]);
        response->marker_id = marker_ids[i];
        constexpr double font_scale = 0.7;
        constexpr int text_thickness = 2;
        int baseline = 0;
        const cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_thickness, &baseline);
        const int text_x = std::clamp(circle_center.x - text_size.width / 2, 0, std::max(0, marker_image_.cols - text_size.width));
        int text_y = circle_center.y - radius - 5;

        if (text_y - text_size.height < 0) {
            text_y = circle_center.y + radius + text_size.height + baseline + 5;
        }
        text_y = std::clamp(text_y, text_size.height, std::max(text_size.height, marker_image_.rows - 1));

        cv::putText(
            marker_image_, label, cv::Point(text_x, text_y), cv::FONT_HERSHEY_SIMPLEX,
            font_scale, cv::Scalar(0, 255, 0), text_thickness, cv::LINE_AA);
    }

    // retransform cv matrix in ros message (image) and publish it on image_with_circle topic
    auto out_msg = cv_bridge::CvImage(header_, "bgr8", marker_image_).toImageMsg();
    //marker_image_pub_->publish(*out_msg);
    marker_image_pub_.publish(*out_msg);

    if (show_image_) {
        cv::imshow("ArUco detection", marker_image_);
        cv::waitKey(1);
    }

    response->success = true;
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg)
  {
    // keep only the header: it is reused for the annotated /marker_image
    header_ = msg->header;
    try {
      // decode the JPEG payload into a BGR OpenCV matrix
      marker_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR(get_logger(), "Could not convert camera image: %s", error.what());
    } catch (const cv::Exception & error) {
      RCLCPP_ERROR(get_logger(), "OpenCV image processing failed: %s", error.what());
    }
  }

  bool show_image_;
  rclcpp::Service<TakePhoto>::SharedPtr service_;
  //rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr marker_image_pub_;
  image_transport::Publisher marker_image_pub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_subscriber_;
  cv::Mat marker_image_;
  std_msgs::msg::Header header_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakePhotoServiceNode>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
