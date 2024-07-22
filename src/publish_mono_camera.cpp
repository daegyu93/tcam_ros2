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
    CameraPublisherLoop(const rclcpp::NodeOptions & options) : Node("camera_publisher", options)
    {
        this->get_parameter("camera_device", camera_device_);
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

            // Sleep for a short duration to prevent CPU overuse
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<CameraPublisherLoop>(options);

    // Start publishing images
    node->publishImages();

    rclcpp::shutdown();
    return 0;
}
