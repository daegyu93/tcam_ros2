#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisherLoop : public rclcpp::Node
{
private:
    cv::VideoCapture cap_;
    image_transport::Publisher publisher_;
    std::string topic_name_;
    int camera_device_;

public:
    CameraPublisherLoop(const rclcpp::NodeOptions & options, int camera_device) : Node("camera_publisher", options) , camera_device_(camera_device)
    {
        cap_.open(camera_device_);

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open video device %d.", camera_device_);
            rclcpp::shutdown();
            return;
        }

        publisher_ = image_transport::create_publisher(this, "/image_raw");
    }

    void publishImages()
    {
        while (rclcpp::ok()) {
            cv::Mat frame;
            cap_ >> frame;

            if (frame.empty()) {
                RCLCPP_WARN(this->get_logger(), "Captured empty frame");
                continue;
            }

            auto header = std_msgs::msg::Header();
            header.stamp = this->get_clock()->now();
            auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            publisher_.publish(*msg);
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "You must specify the camera device as an argument");
        return 1;
    }

    int camera_device = std::stoi(argv[1]);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CameraPublisherLoop>(options, camera_device);

    node->publishImages();

    rclcpp::shutdown();
    return 0;
}
