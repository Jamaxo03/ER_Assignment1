#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "image_transport/transport_hints.hpp"
#include "opencv2/aruco.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ArucoImageDetector
{
public:
  explicit ArucoImageDetector(rclcpp::Node::SharedPtr node)
  : node_(std::move(node)), image_transport_(node_)
  {
    const auto image_topic =
      node_->declare_parameter<std::string>("image_topic", "/camera/image");
    const auto image_transport_name =
      node_->declare_parameter<std::string>("image_transport", "raw");
    show_image_ = node_->declare_parameter<bool>("show_image", true);

    dictionary_ =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    detector_parameters_ = cv::aruco::DetectorParameters::create();

    if (show_image_) {
      cv::namedWindow("ArUco detection", cv::WINDOW_AUTOSIZE);
    }

    transport_hints_ =
      std::make_unique<image_transport::TransportHints>(node_.get());
    subscriber_ = image_transport_.subscribe(
      image_topic, 1,
      std::bind(
        &ArucoImageDetector::image_callback, this, std::placeholders::_1),
      image_transport::ImageTransport::VoidPtr(), transport_hints_.get());

    RCLCPP_INFO(
      node_->get_logger(), "Listening for camera images on '%s' (%s transport)",
      image_topic.c_str(), image_transport_name.c_str());
  }

  ~ArucoImageDetector()
  {
    if (show_image_) {
      cv::destroyWindow("ArUco detection");
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    try {
      // Copy before drawing so the camera message remains unchanged.
      cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
      cv::Mat gray_image;
      cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

      std::vector<int> marker_ids;
      std::vector<std::vector<cv::Point2f>> marker_corners;
      cv::aruco::detectMarkers(
        gray_image, dictionary_, marker_corners, marker_ids, detector_parameters_);

      if (!marker_ids.empty()) {
        std::ostringstream detected_ids;
        for (size_t i = 0; i < marker_ids.size(); ++i) {
          if (i != 0) {
            detected_ids << ", ";
          }
          detected_ids << marker_ids[i];
        }
        RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "Detected ArUco marker ID(s): %s", detected_ids.str().c_str());
        cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
      }

      const cv::Point center(image.cols / 2, image.rows / 2);
      cv::circle(image, center, 50, cv::Scalar(0, 0, 255), 3);

      if (show_image_) {
        cv::imshow("ArUco detection", image);
        cv::waitKey(1);
      }
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(
        node_->get_logger(), "Failed to convert camera image: %s", e.what());
    }
  }

  rclcpp::Node::SharedPtr node_;
  image_transport::ImageTransport image_transport_;
  std::unique_ptr<image_transport::TransportHints> transport_hints_;
  image_transport::Subscriber subscriber_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> detector_parameters_;
  bool show_image_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("aruco_image_detector");
  auto detector = std::make_shared<ArucoImageDetector>(node);
  rclcpp::spin(node);
  detector.reset();
  rclcpp::shutdown();
  return 0;
}
