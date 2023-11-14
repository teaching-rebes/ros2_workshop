#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using std::placeholders::_1;
 
#define SUB_CAMERA                          "/pylon_camera_node/pylon_ros2_camera_node/image_raw" 
using namespace std;
using namespace std::chrono_literals;
 
class LineDetect: public rclcpp::Node {
public:
  LineDetect() : Node("line_detect"), count_(0) {
    // Publisher which publishes a new image whenever it is available
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("line_image", 10);

    // Subscriber which receives the input image
    image_transport::TransportHints th(this);
    std::string ret = th.getTransport();
    subscriber_ =
        image_transport::create_subscription(this, SUB_CAMERA, std::bind(&LineDetect::image_callback, this, _1), ret, rmw_qos_profile_sensor_data);

    // uncomment if timer-based updates are required
    // timer_ = this->create_wall_timer(
    //    500ms, std::bind(&LineDetect::timer_callback, this));
  }
 
private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    // convert received image to OpenCV image
    std::shared_ptr<cv_bridge::CvImage> current_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat can_image, res_image;

    // Canny Edge detection
    cv::Canny(current_image->image, can_image, 50, 400, 3);

    // Standard Hough Line Transformation
    vector<cv::Vec4i> lines;
    cv::HoughLinesP(can_image, lines, 1, CV_PI/180, 50, 50, 10);

    // convert edges to image that will be used to show the detected lines
    cv::cvtColor(can_image, res_image, cv::COLOR_GRAY2BGR);

    // Draw the detected lines
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( res_image, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
    }

    // Convert the OpenCV image to a ROS message that can be sent
    msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", res_image).toImageMsg();
 
 
    // Publish the image to the topic defined in the publisher
    publisher_->publish(*msg_.get());
    auto& clk = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), clk, 5000, "Image %ld published", count_);
    count_++;
  }
  // rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  image_transport::Subscriber subscriber_;
  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<LineDetect>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
