#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class FakePosePub : public rclcpp::Node{
    public:
        FakePosePub() : Node("fake_pose_pub"){
            pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/localization/pose", 10);

            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&FakePosePub::onTimer, this)
            );

            RCLCPP_INFO(get_logger(), "FakePosePub started. Publishing to /localization/pose");
        }

    private:
        void onTimer(){
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "map";

            // 차량 위치 (0,0) 고정
            pose.pose.position.x = 0.0;
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1.0;

            pub_->publish(pose);
        }

        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakePosePub>());
    rclcpp::shutdown();
    return 0;
}