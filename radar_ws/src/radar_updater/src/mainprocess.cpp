#include "radar_updater.hpp"

namespace radar {

class MainProcess : public rclcpp::Node {
public:
    MainProcess(std::string name)
        : Node(name)
    {
        radar_updater = std::make_unique<radar::RadarUpdater>();
        position_subscription_
            = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "car_positions", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                    radar_updater->radar_points_receiver_.receive_enemy_data(msg);
                });
        using namespace std::chrono_literals;
        updater_timer = this->create_wall_timer(
            20ms, [this]() { radar_updater->update_radar_status(); });
    }

private:
    std::unique_ptr<radar::RadarUpdater> radar_updater;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>> position_subscription_;
    std::shared_ptr<rclcpp::TimerBase> updater_timer;
};
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radar::MainProcess>("serial_main_progress"));
    rclcpp::shutdown();

    return 0;
}
