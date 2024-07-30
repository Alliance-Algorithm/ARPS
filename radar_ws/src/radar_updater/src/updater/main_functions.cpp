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

    if (radar_information_->is_active_big_buff_ || radar_information_->enemy_sentry_hp_ < 380)
        if (!radar_information_->is_double_debuff_enabled_ && radar_information_->double_debuff_chances_ != 0)
            radar_information_->double_debuff_cmd_++;

    if (radar_information_->time_remain_ < 30) {
        radar_information_->double_debuff_cmd_++;
    }
}
}