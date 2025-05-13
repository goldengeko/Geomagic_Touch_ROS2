#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "omni_msgs/msg/omni_state.hpp"
#include "omni_msgs/msg/omni_button_event.hpp"

class Omni_Broadcaster : public rclcpp::Node {
public:
    Omni_Broadcaster() : Node("omni_broadcaster") {
        init();
    }

private:
    void init() {
        framename_ = declare_parameter<std::string>("framename", "frame");
        parent_frame_ = declare_parameter<std::string>("parent_frame", "base_link");
        timer_period_ms_ = declare_parameter<int>("timer_period_ms", 100);
        enable_verbose_logging_ = declare_parameter<bool>("verbose_logging", false);

        if (framename_.empty()) {
            RCLCPP_ERROR(get_logger(), "Frame name cannot be empty");
            throw std::runtime_error("Invalid frame name");
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/phantom/pose", 10, std::bind(&Omni_Broadcaster::stateCallback, this, std::placeholders::_1));
        sub_button_ = create_subscription<omni_msgs::msg::OmniButtonEvent>(
            "/phantom/button", 10, std::bind(&Omni_Broadcaster::buttonCallback, this, std::placeholders::_1));

        broadcast_timer_ = create_wall_timer(
            std::chrono::milliseconds(timer_period_ms_),
            std::bind(&Omni_Broadcaster::positionBroadcaster, this));

        RCLCPP_INFO(get_logger(), "Omni_Broadcaster initialized");
    }

    void stateCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> state_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        actTouchPose_ = *state_msg;
        has_pose_ = true;
        if (enable_verbose_logging_) {
            RCLCPP_INFO(get_logger(), "Received Pose: x=%.2f, y=%.2f, z=%.2f, rx=%.2f, ry=%.2f, rz=%.2f, rw=%.2f",
                        actTouchPose_.pose.position.x, actTouchPose_.pose.position.y, actTouchPose_.pose.position.z,
                        actTouchPose_.pose.orientation.x, actTouchPose_.pose.orientation.y,
                        actTouchPose_.pose.orientation.z, actTouchPose_.pose.orientation.w);
        }
    }

    void buttonCallback(const std::shared_ptr<omni_msgs::msg::OmniButtonEvent> button_msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        actbutton_ = *button_msg;
        has_button_ = true;
        if (enable_verbose_logging_) {
            RCLCPP_INFO(get_logger(), "Received Button Event: grey_button=%d, white_button=%d",
                        actbutton_.grey_button, actbutton_.white_button);
        }
    }

    void positionBroadcaster() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (has_pose_ && has_button_ && actbutton_.grey_button == 1 && actbutton_.white_button == 1) {
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = get_clock()->now();
            t.header.frame_id = parent_frame_;
            t.child_frame_id = framename_;
            t.transform.translation.x = actTouchPose_.pose.position.x;
            t.transform.translation.y = actTouchPose_.pose.position.y;
            t.transform.translation.z = actTouchPose_.pose.position.z;
            t.transform.rotation = actTouchPose_.pose.orientation;

            tf_broadcaster_->sendTransform(t);

            if (enable_verbose_logging_) {
                RCLCPP_INFO(get_logger(), "Broadcasting transform: [%.2f, %.2f, %.2f]",
                            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z);
            }
        }
    }

    geometry_msgs::msg::PoseStamped actTouchPose_;
    omni_msgs::msg::OmniButtonEvent actbutton_;
    bool has_pose_ = false;
    bool has_button_ = false;
    std::mutex data_mutex_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<omni_msgs::msg::OmniButtonEvent>::SharedPtr sub_button_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    std::string framename_;
    std::string parent_frame_;
    int timer_period_ms_;
    bool enable_verbose_logging_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Omni_Broadcaster>());
    rclcpp::shutdown();
    return 0;
}