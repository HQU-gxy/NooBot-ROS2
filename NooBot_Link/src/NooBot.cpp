#include "noobot/NooBot.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);									
    rclcpp::spin(std::make_shared<NooBot>());
    rclcpp::shutdown();
    return 0;
}