#pragma once

#include "../include/Logger.hpp"
#include "../include/radar_informations.hpp"

#include <map>
#include <memory>
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

#define RED 100
#define BLUE 101

namespace radar {

class RefereeSerialSubmitter {
public:
    std::shared_ptr<Logger> logger_;
    std::shared_ptr<serial::Serial> serial_;
    std::shared_ptr<RadarInformation> radar_information_;
    std::chrono::steady_clock::time_point time;

    RefereeSerialSubmitter() {};
    RefereeSerialSubmitter(std::shared_ptr<RadarInformation> radar_information, std::shared_ptr<Logger> logger, std::shared_ptr<serial::Serial> serial)
        : logger_(logger)
        , serial_(serial)
        , radar_information_(radar_information) {};

    void send_serial_data()
    {
        if (!serial_->isOpen())
            return;

        for (size_t i = 0; i < radar_information_->enemy_robot_positions.size(); i++) {
            uint8_t enemy_position_data[19];
            enemy_position_data_pack_old(enemy_position_data, radar_information_->enemy_robot_positions.find(i)->second, i, 0);
            serial_util::dji_crc::append_crc16(enemy_position_data);
            // 串口数据发送
            serial_->write(enemy_position_data, sizeof(enemy_position_data));
        }

        uint8_t radar_decision_data[10];
        radar_cmd_data_pack(radar_decision_data, radar_information_->double_debuff_cmd_, 0);
        serial_util::dji_crc::append_crc16(radar_decision_data);
        serial_->write(radar_decision_data, sizeof(radar_decision_data));
    }

    /*
     * 串口数据打包(旧)
     * @param emeny_robot_positions 敌方机器人位置信息
     * @param req 包序号
     * @return serial_data 串口数据
     */
    void enemy_position_data_pack_old(uint8_t* serial_data, enemy_robot_position one_enemy_robot_position, int index, int req)
    {
        map_robot_data_t enemy_robot_position_to_send = map_robot_data_t {
            static_cast<uint16_t>(index),
            one_enemy_robot_position.x,
            one_enemy_robot_position.y
        };
        // 帧头
        uint8_t frame_header[5]
            = { 0xA5, 0x0A, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
        serial_util::dji_crc::append_crc8(frame_header);

        for (int i = 0; i < 5; i++)
            serial_data[i] = frame_header[i];
        reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0305;

        reinterpret_cast<map_robot_data_t&>(serial_data[7]) = enemy_robot_position_to_send;
    }
    /*
     * 串口数据打包(新)
     * @param emeny_robot_positions 敌方机器人位置信息
     * @param req 包序号
     * @return serial_data 串口数据
     */
    // void enemy_position_data_pack(uint8_t* serial_data, std::map<int, enemy_robot_position> emeny_robot_positions, int req)
    // {
    // }

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
};
}