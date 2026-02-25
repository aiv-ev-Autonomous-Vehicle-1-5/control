#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <limits>

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

        // 1. 현재 위치
        double px = current_pose_.pose.position.x;
        double py = current_pose_.pose.position.y;

        // 2. 가장 가까운 점 찾기
        double min_dist = std::numeric_limits<double>::max();
        int closest_index = -1;

        for(size_t i = 0; i < current_path_.poses.size(); ++i){
            double dx = current_path_.poses[i].pose.position.x - px;
            double dy = current_path_.poses[i].pose.position.y -py;
            double dist = std::sqrt(dx * dx + dy * dy);

            if(dist < min_dist){
                min_dist = dist;
                closest_index = i;
            }
        }

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Closest index: %d, distance: %.3f",
            closest_index,
            min_dist
        );

        // Lookahead distance
        double Ld = 1.0;  // 1m

        double accumulated_dist = 0.0;
        int target_index = closest_index;

        for (size_t i = closest_index; i < current_path_.poses.size() - 1; ++i) {

            double x1 = current_path_.poses[i].pose.position.x;
            double y1 = current_path_.poses[i].pose.position.y;

            double x2 = current_path_.poses[i+1].pose.position.x;
            double y2 = current_path_.poses[i+1].pose.position.y;

            double segment_dist = std::sqrt(
                (x2 - x1)*(x2 - x1) +
                (y2 - y1)*(y2 - y1)
            );

            accumulated_dist += segment_dist;

            if (accumulated_dist >= Ld) {
                target_index = i + 1;
                break;
            }   
        }   

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "Target index: %d",
            target_index
        );


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