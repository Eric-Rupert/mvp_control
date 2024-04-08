
#include "mvp_control/mvp_control_ros.hpp"

#include "iostream"

int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    // ctrl::MvpControlROS control_ros;
    auto control_ros = std::make_shared<ctrl::MvpControlROS>();
    
    control_ros->initialize();

    rclcpp::spin(control_ros);

    rclcpp::shutdown();

    return 0;

}