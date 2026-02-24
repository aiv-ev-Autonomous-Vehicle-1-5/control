#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PathFollower : public rclcpp::Node{
    public:
        PathFollower() : Node("path_follower_node"){
            RCLCPP_INFO(this->get_logger(), "Path Follower Node Started");

            path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                "/planning/path",
                10,
                std::bind(&PathFollower::onPath, this, std::placeholders::_1)
            );

            pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/localization/pose",
                10,
                std::bind(&PathFollower::onPose, this, std::placeholders::_1)
            );

            // 더미 출력용 cmd_vel publisher
            cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(50), // 20Hz
                std::bind(&PathFollower::controlLoop, this)
            );
    }

    private:
        void onPath(const nav_msgs::msg::Path::SharedPtr msg){
            current_path_ = *msg;
            has_path_ = true;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Path received: poses=%zu",
                msg->poses.size()
            );
        }

        void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            current_pose_ = *msg;
            has_pose_ = true;

            RCLCPP_INFO_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Pose received: x=%.2f y=%.2f",
                msg->pose.position.x,
                msg->pose.position.y
            );
        }

    void controlLoop(){
        if (!has_path_ || !has_pose_) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for path...");
        return;
    }

        // 아직 제어 계산 안 함: 더미 cmd_vel만 publish
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.5;   // 전진 0.5 m/s (더미)
        cmd.angular.z = 0.0;  // 회전 0.0 rad/s

        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Control running with pose"
        );
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    geometry_msgs::msg::PoseStamped current_pose_;
    bool has_pose_{false};

    nav_msgs::msg::Path current_path_;
    bool has_path_{false};
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}