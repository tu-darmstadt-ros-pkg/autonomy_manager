//
// Created by aljoscha on 20.02.24.
//
#include <autonomy_manager/autonomy_manager.h>

using namespace autonomy_manager;

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomyManager>());
    rclcpp::shutdown();
    return 0;
}