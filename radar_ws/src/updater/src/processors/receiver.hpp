#pragma once

#include "../include/received_packages.hpp"
#include "submitter.hpp"

#include <bitset>
#include <chrono>
#include <rcutils/types/rcutils_ret.h>
#include <vector>

#define RED 100
#define BLUE 101

namespace radar {

class EnemyPointsReceiver {
public:
    std::shared_ptr<Logger> logger_;
    Configs radar_config_;
    std::shared_ptr<RadarInformation> radar_information_;

    EnemyPointsReceiver() {};
    EnemyPointsReceiver(
        std::shared_ptr<RadarInformation> radar_information,
        std::shared_ptr<Logger> logger,
        Configs radar_config)
        : logger_(logger)
        , radar_config_(radar_config)
        , radar_information_(radar_information) {};

public:
    void receive_enemy_data(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        try {
            // logger_.INFO("-->Received data from car_detect(size:" + std::to_string(msg->data.size()) + ")");

            // 串口数据解析
            if (msg->data.size() != 0) {
                for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
                    map_robot_data_t temp_position;

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
                        it->second = enemy_robot_position {
                            temp_position.target_position_x, temp_position.target_position_y, std::chrono::steady_clock::now()
                        };
                    } else {
                        radar_information_->enemy_robot_positions.insert(
                            std::make_pair(int(temp_position.target_robot_id),
                                enemy_robot_position {
                                    temp_position.target_position_x, temp_position.target_position_y, std::chrono::steady_clock::now() }));
                    }
                }
            }

        } catch (const std::exception& e) {
            logger_->ERRORS("[x]Error in receive points data: " + std::string(e.what()));
        }
    }
};

class RefereeSerialReceiver {
public:
    Configs radar_config_;

    package::receive::Frame frame_;
    std::shared_ptr<serial::Serial> serial_;
    size_t cache_size_ = 0;

    std::shared_ptr<Logger> logger_;
    std::shared_ptr<RadarInformation> radar_information_;

    RefereeSerialReceiver() {};
    RefereeSerialReceiver(
        std::shared_ptr<RadarInformation> radar_information,
        std::shared_ptr<serial::Serial> serial,
        std::shared_ptr<Logger> logger,
        Configs radar_config)
        : radar_config_(radar_config)
        , serial_(serial)
        , logger_(logger)
        , radar_information_(radar_information)
    {
        logger->INFO("[√]initialized serial node.");
    };

public:
    void receive_referee_data()
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
                    logger_->WARNING("Body crc16 invalid");
                }
            }
        } else {
            auto result = serial_util::receive_package(
                *serial_, frame_.header, cache_size_,
                static_cast<uint8_t>(0xa5),
                [](const package::receive::FrameHeader& header) {
                    return serial_util::dji_crc::verify_crc8(header);
                });
            if (result == serial_util::ReceiveResult::HEADER_INVAILD) {
                logger_->WARNING("Header start invalid");
            } else if (result == serial_util::ReceiveResult::VERIFY_INVAILD) {
                logger_->WARNING("Header crc8 invalid");
            }
        }
    }

    void process_frame_info()
    {
        auto command_id = frame_.body.command_id;
        RCLCPP_INFO(rclcpp::get_logger("Referee serial"), "Referee command id: %x", command_id);
        RCLCPP_INFO(rclcpp::get_logger("Radar cmd"), "Radar CMD: %d", radar_information_->double_debuff_cmd_);

        if (command_id == 0x0001)
            update_game_status();
        if (command_id == 0x0003)
            update_sentry_hp();
        if (command_id == 0x0101)
            update_buff_info();
        if (command_id == 0x020E)
            update_radar_info();
        if (command_id == 0x0301)
            update_enemy_status_by_sentry();
    }

    void update_game_status()
    {
        auto& data = reinterpret_cast<package::receive::GameStatus&>(frame_.body.data);

        radar_information_->gamestate_ = static_cast<GameState>(data.game_stage);
        radar_information_->time_remain_ = static_cast<int>(data.stage_remain_time);
        logger_->INFO("---> receive gamestate:" + std::to_string(static_cast<int>(radar_information_->gamestate_))
            + "|time_remain:" + std::to_string(radar_information_->time_remain_));
    };
    void update_sentry_hp()
    {
        auto& data = reinterpret_cast<package::receive::GameRobotHp&>(frame_.body.data);

        radar_information_->enemy_sentry_hp_ = radar_information_->friend_Side_ == RED ? data.blue_7 : data.red_7;
        logger_->INFO("receive enemy sentry hp:" + std::to_string(radar_information_->enemy_sentry_hp_));
    };
    void update_buff_info()
    {
        auto& data = reinterpret_cast<package::receive::EventData&>(frame_.body.data);
        std::bitset<32> event_info_val(data.event_data);

        radar_information_->is_active_big_buff_ = static_cast<bool>(event_info_val[5]);
        logger_->INFO("receive big buff info:" + std::to_string(radar_information_->is_active_big_buff_));
    };
    void update_radar_info()
    {
        auto& data = reinterpret_cast<package::receive::RadarInfo&>(frame_.body.data);

        auto bit_0 = (data.radar_info >> 7) % 2;
        auto bit_1 = (data.radar_info >> 6) % 2;
        auto bit_2 = (data.radar_info >> 5) % 2;

        int current_double_debuff_chances = bit_0 * 2 + bit_1;
        logger_->INFO("receive double debuff chances: " + std::to_string(current_double_debuff_chances));
        if (current_double_debuff_chances > radar_information_->double_debuff_chances_) {
            logger_->INFO("double debuff chances add:" + std::to_string(radar_information_->double_debuff_chances_));
        }
        radar_information_->double_debuff_chances_ = current_double_debuff_chances;

        radar_information_->is_double_debuff_enabled_ = static_cast<bool>(bit_2);
        logger_->INFO("[receive info]is doublebuff enabled:" + std::to_string(radar_information_->is_double_debuff_enabled_));
    };
    void update_enemy_status_by_sentry()
    {
        auto& data = reinterpret_cast<package::receive::DataFromSentry&>(frame_.body.data);
        if (data.sender_id != (radar_config_.friend_side == RED ? 7 : 107) || data.receiver_id != (radar_config_.friend_side == RED ? 9 : 109) || data.data_cmd_id != 0x0222)
            return;

        auto enemy_robot_positions_received_from_sentry = reinterpret_cast<std::vector<enemy_robot_position_to_sentry>&>(data.user_data);

        for (int i = 0; i < static_cast<int>(enemy_robot_positions_received_from_sentry.size()); i++) {
            if (enemy_robot_positions_received_from_sentry[i].x == -114514 && enemy_robot_positions_received_from_sentry[i].y == -114514)
                continue;

            auto it = radar_information_->enemy_robot_positions.find(enemy_robot_positions_received_from_sentry[i].id);
            if (it != radar_information_->enemy_robot_positions.end()) {
                it->second.x = enemy_robot_positions_received_from_sentry[i].x;
                it->second.y = enemy_robot_positions_received_from_sentry[i].y;
                it->second.last_updated_time = std::chrono::steady_clock::now();
            }
        }
    };
};
}