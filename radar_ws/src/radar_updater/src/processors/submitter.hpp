#pragma once

#include "../include/Logger.hpp"
#include "../include/radar_informations.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <opencv2/core/types.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <unistd.h>

#include <opencv2/core.hpp>

#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <serial_util/package_receive.hpp>

#include <rcl/event.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <utility>
#include <vector>

#define RED 100
#define BLUE 101

namespace radar {

class RefereeSerialSubmitter {
public:
    std::shared_ptr<Logger> logger_;
    std::shared_ptr<serial::Serial> serial_;
    std::shared_ptr<RadarInformation> radar_information_;
    std::chrono::steady_clock::time_point time;
    Configs radar_config_;

    RefereeSerialSubmitter() {};
    RefereeSerialSubmitter(std::shared_ptr<RadarInformation> radar_information, std::shared_ptr<Logger> logger, std::shared_ptr<serial::Serial> serial, Configs radar_config)
        : logger_(logger)
        , serial_(serial)
        , radar_information_(radar_information)
        , radar_config_(radar_config)
    {
        logger_->INFO("[√]initialized submitter node.");
    };

    void send_serial_data()
    {
        if (!serial_->isOpen())
            return;
        uint8_t enemy_position_data[33];
        enemy_position_data_pack(enemy_position_data, radar_information_->enemy_robot_positions, 0);
        serial_util::dji_crc::append_crc16(enemy_position_data);
        serial_->write(enemy_position_data, sizeof(enemy_position_data));

        // uint8_t enemy_position_data_to_sentry[69];
        // enemy_position_data_pack_to_sentry(enemy_position_data_to_sentry, radar_information_->enemy_robot_positions, 0);
        // serial_util::dji_crc::append_crc16(enemy_position_data_to_sentry);
        // serial_->write(enemy_position_data_to_sentry, sizeof(enemy_position_data_to_sentry));

        uint8_t radar_decision_data[10];
        radar_cmd_data_pack(radar_decision_data, radar_information_->double_debuff_cmd_, 0);
        serial_util::dji_crc::append_crc16(radar_decision_data);
        serial_->write(radar_decision_data, sizeof(radar_decision_data));
    }

    /*
     * 串口数据打包(新)
     * @param enemy_robot_positions 敌方机器人位置信息
     * @param req 包序号
     * @return serial_data 串口数据
     */
    void enemy_position_data_pack(uint8_t* serial_data, std::map<int, enemy_robot_position> enemy_robot_positions, int req)
    {
        std::vector<cv::Point2d> enemy_positions(6, cv::Point2d(0, 0));
        for (int i = 0; i < 6; i++) {

            auto it = enemy_robot_positions.find(radar_information_->catorgories[i]);
            if (it != enemy_robot_positions.end()) {
                enemy_positions[i] = cv::Point2d(it->second.x, it->second.y);
            }
        }

        enemy_robot_position_new enemy_robot_position_to_send = enemy_robot_position_new {
            static_cast<uint16_t>(enemy_positions[0].x),
            static_cast<uint16_t>(enemy_positions[0].y),
            static_cast<uint16_t>(enemy_positions[1].x),
            static_cast<uint16_t>(enemy_positions[1].y),
            static_cast<uint16_t>(enemy_positions[2].x),
            static_cast<uint16_t>(enemy_positions[2].y),
            static_cast<uint16_t>(enemy_positions[3].x),
            static_cast<uint16_t>(enemy_positions[3].y),
            static_cast<uint16_t>(enemy_positions[4].x),
            static_cast<uint16_t>(enemy_positions[4].y),
            static_cast<uint16_t>(enemy_positions[5].x),
            static_cast<uint16_t>(enemy_positions[5].y)
        };

        // 帧头
        uint8_t frame_header[5]
            = { 0xA5, 0x0A, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
        serial_util::dji_crc::append_crc8(frame_header);

        for (int i = 0; i < 5; i++)
            serial_data[i] = frame_header[i];
        reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0305;

        reinterpret_cast<enemy_robot_position_new&>(serial_data[7]) = enemy_robot_position_to_send;
    }

    /*
     * 串口数据打包
     * @param double_debuff_cmd 雷达决策命令
     * @param req 包序号
     * @return serial_data 串口数据
     */
    void radar_cmd_data_pack(uint8_t* serial_data, int double_debuff_cmd, int req)
    {
        // 帧头
        uint8_t frame_header[5] = { 0xA5, 0x0A, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
        serial_util::dji_crc::append_crc8(frame_header);

        for (int i = 0; i < 5; i++)
            serial_data[i] = frame_header[i];

        reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0121;

        reinterpret_cast<int&>(serial_data[7]) = double_debuff_cmd;
    }

    /*
     * 给哨兵的串口数据打包
     * @param enemy_robot_positions 敌方机器人位置信息
     * @param req 包序号
     * @return serial_data 串口数据
     */
    // void enemy_position_data_pack_to_sentry(uint8_t* serial_data, std::map<int, enemy_robot_position> enemy_robot_positions, int req)
    // {
    //     // 帧头
    //     uint8_t frame_header[5]
    //         = { 0xA5, 0x0A, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
    //     serial_util::dji_crc::append_crc8(frame_header);

    //     for (int i = 0; i < 5; i++)
    //         serial_data[i] = frame_header[i];
    //     reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0305;

    //     // 与哨兵的约定id : 0x0222
    //     reinterpret_cast<uint16_t&>(serial_data[7]) = 0x0222;
    //     reinterpret_cast<uint16_t&>(serial_data[9]) = radar_config_.friend_side == RED ? 9 : 109;
    //     reinterpret_cast<uint16_t&>(serial_data[11]) = radar_config_.friend_side == RED ? 7 : 107;

    //     for (int i = 0; i < 6; i++) {
    //         enemy_robot_position_to_sentry single_enemy_position;
    //         single_enemy_position.id = (radar_config_.friend_side == RED ? i + 101 : i + 1);

    //         auto it = enemy_robot_positions.find(i);
    //         if (it != enemy_robot_positions.end()) {
    //             single_enemy_position.x = it->second.x;
    //             single_enemy_position.y = it->second.y;
    //         } else {
    //             single_enemy_position.x = -404;
    //             single_enemy_position.y = -404;
    //         }
    //         reinterpret_cast<enemy_robot_position_to_sentry&>(serial_data[13 + (i * sizeof(single_enemy_position))]) = single_enemy_position;
    //     }
    // }
};
}