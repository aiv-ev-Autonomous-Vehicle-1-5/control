#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class FakePathPub : public rclcpp::Node{
    public:
        FakePathPub() : Node("fake_path_pub"){
            pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/path", 10);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(&FakePathPub::onTimer, this)
            );
            RCLCPP_INFO(get_logger(), "FakePathPub started. Publishing to /planning/path");
    }

    private:
        void onTimer(){
            nav_msgs::msg::Path path;
            path.header.stamp = this->now();
            path.header.frame_id = "map";

            // 직선 경로(0,0) -> (9,0)
            for (int i = 0; i < 10; ++i) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header = path.header;
                ps.pose.position.x = static_cast<double>(i);
                ps.pose.position.y = 0.0;
                ps.pose.orientation.w = 1.0;
                path.poses.push_back(ps);
            }

            pub_->publish(path);
        }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FakePathPub>());
    rclcpp::shutdown();
    return 0;
}