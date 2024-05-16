#include "radar_serial_processor.hpp"

// 串口发送数据
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar::MainProcess>("serial_main_progress"));
    rclcpp::shutdown();

    return 0;
}
