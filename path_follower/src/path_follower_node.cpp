#include "rclcpp/rclcpp.hpp"

class PathFollower : public rclcpp::Node{
    public:
        PathFollower() : Node("path_follower_node"){
            RCLCPP_INFO(this->get_logger(), "Path Follower Node Started");
        }
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathFollower>());
    rclcpp::shutdown();
    return 0;
}