#include "../updater.hpp"

namespace radar {

void Updater::receive_topic_data(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // TODO: rewrite the receive_topic_data method
    try {
        // 串口数据解析
        if (msg->data.size() != 0) {
            for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
                info::single_robot_position_data temp_position;

                temp_position.target_robot_id = msg->data.data()[i];
                if (radar_config_.friend_side == BLUE) {
                    temp_position.target_position_x = msg->data.data()[i + 1];
                    temp_position.target_position_y = msg->data.data()[i + 2];
                } else {
                    temp_position.target_position_x = 28 - msg->data.data()[i + 1];
                    temp_position.target_position_y = 15 - msg->data.data()[i + 2];
                }
                auto it = radar_information_->enemy_robot_positions.find(int(temp_position.target_robot_id));
                if (it != radar_information_->enemy_robot_positions.end()) {
                    it->second = info::enemy_robot_position {
                        temp_position.target_position_x, temp_position.target_position_y, std::chrono::steady_clock::now()
                    };
                } else {
                    radar_information_->enemy_robot_positions.insert(
                        std::make_pair(int(temp_position.target_robot_id),
                            info::enemy_robot_position {
                                temp_position.target_position_x, temp_position.target_position_y, std::chrono::steady_clock::now() }));
                }
            }
        }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "[x]Error in receive points data: %s", e.what());
    }
}

void Updater::receive_referee_data()
{
    if (!serial_->isOpen())
        return;

    if (cache_size_ >= sizeof(frame_.header)) {
        auto frame_size = sizeof(frame_.header) + sizeof(frame_.body.command_id) + frame_.header.data_length + sizeof(uint16_t);
        cache_size_ += serial_->read(reinterpret_cast<uint8_t*>(&frame_) + cache_size_,
            frame_size - cache_size_);

        if (cache_size_ == frame_size) {
            cache_size_ = 0;
            if (serial_util::dji_crc::verify_crc16(&frame_, frame_size)) {
                process_frame_info();
            } else {
                RCLCPP_WARN(get_logger(), "Body crc16 invalid");
            }
        }
    } else {
        auto result = serial_util::receive_package(
            *serial_, frame_.header, cache_size_,
            static_cast<uint8_t>(0xa5),
            [](const package::receive::FrameHeader& header) {
                return serial_util::dji_crc::verify_crc8(header);
            });
        if (result == serial_util::ReceiveResult::HEADER_INVALID) {
            RCLCPP_WARN(get_logger(), "Header start invalid");
        } else if (result == serial_util::ReceiveResult::HEADER_INVALID) {
            RCLCPP_WARN(get_logger(), "Header crc8 invalid");
        }
    }
}

void Updater::process_frame_info()
{
    auto command_id = frame_.body.command_id;
    // RCLCPP_INFO(get_logger(), "* Receive [0x0%x]", command_id);

    if (command_id == 0x0001) {
        update_game_status();
        RCLCPP_INFO(get_logger(), "---> Radar command ID [%d]", radar_information_->double_debuff_cmd_);
    }
    if (command_id == 0x0003)
        update_sentry_hp();
    if (command_id == 0x0101)
        update_buff_info();
    if (command_id == 0x020E)
        update_radar_info();
    if (command_id == 0x0301)
        update_enemy_status_by_sentry();
}

void Updater::update_game_status()
{
    auto& data = reinterpret_cast<package::receive::GameStatus&>(frame_.body.data);

    radar_information_->gamestate_ = static_cast<info::GameState>(data.game_stage);
    radar_information_->time_remain_ = static_cast<int>(data.stage_remain_time);
    RCLCPP_INFO(get_logger(), "--->[0x0001] receive gamestate:%d|time_remain:%d",
        static_cast<int>(radar_information_->gamestate_), radar_information_->time_remain_);
};
void Updater::update_sentry_hp()
{
    auto& data = reinterpret_cast<package::receive::GameRobotHp&>(frame_.body.data);

    radar_information_->enemy_sentry_hp_ = radar_information_->friend_Side_ == RED ? data.blue_7 : data.red_7;
    RCLCPP_INFO(get_logger(), "--->[0x0003] receive enemy sentry hp:%f", radar_information_->enemy_sentry_hp_);
};
void Updater::update_buff_info()
{
    auto& data = reinterpret_cast<package::receive::EventData&>(frame_.body.data);
    std::bitset<32> event_info_val(data.event_data);

    radar_information_->is_active_big_buff_ = static_cast<bool>(event_info_val[5]);
    RCLCPP_INFO(get_logger(), "--->[0x0101] receive big buff info:%d", radar_information_->is_active_big_buff_);
};
void Updater::update_radar_info()
{
    auto& data = reinterpret_cast<package::receive::RadarInfo&>(frame_.body.data);

    auto bit_0 = (data.radar_info >> 7) % 2;
    auto bit_1 = (data.radar_info >> 6) % 2;
    auto bit_2 = (data.radar_info >> 5) % 2;

    int current_double_debuff_chances = bit_0 * 2 + bit_1;
    RCLCPP_INFO(get_logger(), "--->[0x020E] receive double debuff chances: %d", current_double_debuff_chances);
    if (current_double_debuff_chances > radar_information_->double_debuff_chances_) {
        RCLCPP_INFO(get_logger(), "--->[0x020E] double debuff chances add:%d", radar_information_->double_debuff_chances_);
    }
    radar_information_->double_debuff_chances_ = current_double_debuff_chances;

    radar_information_->is_double_debuff_enabled_ = static_cast<bool>(bit_2);
    RCLCPP_INFO(get_logger(), "--->[0x020E] receive doublebuff if enabled:%d", radar_information_->is_double_debuff_enabled_);
};

void Updater::update_enemy_status_by_sentry()
{
    auto& data = reinterpret_cast<package::receive::DataFromSentry&>(frame_.body.data);
    if (data.sender_id != (radar_config_.friend_side == RED ? 7 : 107) || data.receiver_id != (radar_config_.friend_side == RED ? 9 : 109) || data.data_cmd_id != 0x0222)
        return;

    auto sensor_data = reinterpret_cast<info::position_data_with_sentry&>(data.user_data);
    RCLCPP_INFO(get_logger(), "--->[0x0301] receive data from sentry.");

    for (int i = 0; i < 6; i++) {
        if (sensor_data.positions[i].x == 0 && sensor_data.positions[i].y == 0)
            continue;

        auto it = radar_information_->enemy_robot_positions.find(radar_information_->enemy_catorgories[i]);
        if (it != radar_information_->enemy_robot_positions.end()) {
            it->second.x = sensor_data.positions[i].x;
            it->second.y = sensor_data.positions[i].y;
            it->second.last_updated_time = std::chrono::steady_clock::now();
        } else {
            radar_information_->enemy_robot_positions.insert(
                std::make_pair(radar_information_->enemy_catorgories[i],
                    info::enemy_robot_position {
                        sensor_data.positions[i].x,
                        sensor_data.positions[i].y,
                        std::chrono::steady_clock::now() }));
        }
    }
};
}