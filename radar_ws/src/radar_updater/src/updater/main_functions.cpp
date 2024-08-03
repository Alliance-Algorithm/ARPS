#include "../updater.hpp"

namespace radar {

void Updater::update_radar_status()
{
    try {
        receive_referee_data();
        process_data();
        submit_serial_data();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
    }
}

void Updater::process_data()
{
    for (auto single_enemy_robot_position : radar_information_->enemy_robot_positions) {
        float time_duration = static_cast<float>(
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - single_enemy_robot_position.second.last_updated_time).count());
        if (time_duration > radar_config_.delay_duration)
            radar_information_->enemy_robot_positions.erase(single_enemy_robot_position.first);
    }
    if (radar_config_.debug)
        plot();
    if (radar_information_->gamestate_ == info::GameState::STARTED) {
        if (radar_information_->enemy_robot_positions.size() > 4) {
            radar_information_->double_debuff_cmd_ < 1 ? radar_information_->double_debuff_cmd_++ : 1;
        }
        if (radar_information_->time_remain_ < 61) {
            radar_information_->double_debuff_cmd_ < 2 ? radar_information_->double_debuff_cmd_++ : 2;
        }
    }
}
}