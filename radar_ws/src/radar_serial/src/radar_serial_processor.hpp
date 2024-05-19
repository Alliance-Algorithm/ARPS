#pragma once

#include "Logger.hpp"
#include "nlohmann/json.hpp"
#include "radar_decisionmaker.hpp"

#include <rcl/event.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>

using json = nlohmann::json;

namespace radar {
// 读取配置文件
struct Configs {
    bool debug;
    int friend_side;
    std::string log_path;
    std::string serial_port;
    float delay_duration;
};
/*
 * 串口数据格式 - 定位数据
 * 帧头(5字节) + cmd id(2字节) + 数据(10字节) + CRC16(2字节)
 * 帧头: SOF(1字节) + 数据长度(2字节) + 包序号(1字节) + CRC8(1字节)
 * 数据: 目标机器人ID(2字节) + 目标机器人x坐标(4字节) + 目标机器人y坐标(4字节)
 * 帧尾: CRC16(2字节)
 */
struct __attribute__((packed)) map_robot_data_t {
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
};

/*
 * 串口数据格式 - 雷达决策数据
 * 帧头(5字节) + cmd id(2字节) + 数据(1字节) + CRC16(2字节)
 * 帧头: SOF(1字节) + 数据长度(2字节) + 包序号(1字节) + CRC8(1字节)
 * 数据: cmd命令(1字节)
 * 帧尾: CRC16(2字节)
 */
struct __attribute__((packed)) radar_cmd_t {
    uint8_t cmd;
};
/* 处理敌方机器人坐标
 */
class Enemy_robot_positions_processor {
public:
    struct Status {
        std::optional<cv::Point2f> position;
        std::chrono::steady_clock::time_point time;
    };
    std::map<int, std::optional<Status>> robot_positions;
    Configs radar_serial_config_;
    Logger* logger_;

    Enemy_robot_positions_processor(Configs radar_serial_config, Logger* logger)
        : radar_serial_config_(radar_serial_config)
        , logger_(logger)

    {
        if (radar_serial_config_.friend_side == RED) {
            for (int i = 1; i < 8; i++) {
                robot_positions.insert(std::make_pair(i,
                    Status { std::nullopt, std::chrono::steady_clock::now() }));
            };
        } else {
            for (int i = 101; i < 108; i++) {
                robot_positions.insert(std::make_pair(i,
                    Status { std::nullopt, std::chrono::steady_clock::now() }));
            };
        }
    };

    std::vector<map_robot_data_t> update_positions(std::vector<map_robot_data_t> enemy_positions)
    {
        std::vector<map_robot_data_t> enemy_positions_processed;

        std::vector<int> avaliable_robots;
        for (unsigned i = 0; i < enemy_positions.size(); i++) {
            avaliable_robots.push_back(static_cast<int>(enemy_positions[i].target_robot_id));
            robot_positions[static_cast<int>(enemy_positions[i].target_robot_id)] = Status {
                cv::Point2f(
                    enemy_positions[i].target_position_x, enemy_positions[i].target_position_y),
                std::chrono::steady_clock::now()
            };
        }
        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        for (auto it : robot_positions) {
            float time_duration = static_cast<float>(
                std::chrono::duration_cast<std::chrono::seconds>(now - it.second->time).count());
            if (time_duration > radar_serial_config_.delay_duration) {
                it.second->position = std::nullopt;
            }
            if (it.second->position != std::nullopt) {
                map_robot_data_t position;
                position.target_robot_id = it.first;
                position.target_position_x = it.second->position.value().x;
                position.target_position_y = it.second->position.value().y;
                enemy_positions_processed.push_back(position);

                // logger_->WARNING("id:" + std::to_string(position.target_robot_id));
            }
        }
        return enemy_positions_processed;
    }
};

class Serial_handler {
public:
    Configs radar_serial_config_;
    std::map<float, std::string> categories_;
    bool debug_;
    Logger logger_;
    cv::Mat minimap_image_;
    cv::Mat temp_map_;
    std::vector<map_robot_data_t> enemy_positions_;

    std::shared_ptr<serial::Serial> radar_serial_;
    radar::DecisionMaker radar_decisioner_;
    Enemy_robot_positions_processor enemy_robot_positions_processor;

public:
    Serial_handler()
        : radar_serial_config_(read_config("./resources/config.json"))
        , categories_({ { 0, "CantIdentfy" }, { 1, "R_Hero" }, { 2, "R_Engineer" },
              { 3, "R_Solider_3" }, { 4, "R_Solider_4" }, { 5, "R_Solider_5" },
              { 7, "R_Sentry" }, { 101, "B_Hero" }, { 102, "B_Engineer" },
              { 103, "B_Solider_3" }, { 104, "B_Solider_4" }, { 105, "B_Solider_5" },
              { 107, "B_Sentry" } })
        , debug_(radar_serial_config_.debug)
        , logger_(Logger(Logger::file, Logger::debug, radar_serial_config_.log_path))
        , minimap_image_(cv::imread("./resources/images/RM2024Battlefield.png"))
        , radar_decisioner_()
        , enemy_robot_positions_processor(radar_serial_config_, &logger_)
    {
        // 串口初始化
        while (true) {
            try {
                radar_serial_ = std::make_shared<serial::Serial>(radar_serial_config_.serial_port, 115200U, serial::Timeout::simpleTimeout(50U), serial::eightbits, serial::parity_none,
                    serial::stopbits_one, serial::flowcontrol_none);
                break;
            } catch (const serial::IOException& se) {
                logger_.ERRORS(se.what());
            }
            sleep(1);
        }

        radar_decisioner_ = radar::DecisionMaker(radar_serial_, &logger_, radar_serial_config_.friend_side);
    }

    Configs read_config(std::string filename)
    {
        std::ifstream i(filename);
        json config_json;
        i >> config_json;

        Configs config;

        config.debug = config_json["SERIAL_DEBUG"].get<std::string>() == "true" ? true : false;
        config.friend_side = config_json["FRIEND_SIDE"].get<std::string>() == "RED" ? RED : BLUE;
        config.log_path = config_json["serial_log_path"].get<std::string>();
        config.serial_port = config_json["serial_port"].get<std::string>();
        config.delay_duration = config_json["delay_duration"].get<float>();
        return config;
    }

    /*
     * 串口数据打包
     * @param emeny_robot_positions 敌方机器人位置信息
     * @param req 包序号
     * @return serial_data 串口数据
     */
    void serial_data_pack(uint8_t* serial_data, map_robot_data_t emeny_robot_positions, int req)
    {
        // 帧头
        uint8_t frame_header[5] = { 0xA5, 0x0A, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
        serial_util::dji_crc::append_crc8(frame_header);

        for (int i = 0; i < 5; i++)
            serial_data[i] = frame_header[i];
        reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0305;

        reinterpret_cast<map_robot_data_t&>(serial_data[7]) = emeny_robot_positions;
    }

    /*
     * 串口数据打包
     * @param double_debuff_cmd 雷达是否确认触发双倍易伤
     * @param req 包序号
     * @return serial_data 串口数据
     */
    void serial_data_pack(uint8_t* serial_data, int double_debuff_cmd, int req)
    {
        // 帧头
        uint8_t frame_header[5] = { 0xA5, 0x0A, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
        serial_util::dji_crc::append_crc8(frame_header);

        for (int i = 0; i < 5; i++)
            serial_data[i] = frame_header[i];

        reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0121;

        reinterpret_cast<int&>(serial_data[7]) = double_debuff_cmd;
    }
    void debug_imageshow(std::vector<map_robot_data_t> enemy_robot_positions)
    {
        for (auto robot_position : enemy_robot_positions) {

            int car_position_x = int((robot_position.target_position_x / 28.0) * 1854.0);
            int car_position_y = int(((15.0 - robot_position.target_position_y) / 15.0) * 996.0);
            cv::Point2d car_point(car_position_x, car_position_y);
            cv::Point2d id_point(car_position_x + 30, car_position_y + 10);

            cv::Scalar color = int(robot_position.target_robot_id) > 7 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

            cv::circle(temp_map_, car_point, 20, color, -1);
            cv::putText(temp_map_, categories_[int(robot_position.target_robot_id)], id_point, 1, 2,
                cv::Scalar(255, 255, 255), 2);
        }
    }

public:
    void send_radar_cmd()
    {
        radar_decisioner_.receive_status_data();
        uint8_t radar_decision_data[10];
        serial_data_pack(radar_decision_data, radar_decisioner_.radar_information_.double_debuff_cmd, 0);
        serial_util::dji_crc::append_crc16(radar_decision_data);
        radar_serial_->write(radar_decision_data, sizeof(radar_decision_data));

        std::vector<map_robot_data_t> enemy_positions_processed = enemy_robot_positions_processor.update_positions(enemy_positions_);
        if (debug_) {
            minimap_image_.copyTo(temp_map_);

            debug_imageshow(enemy_positions_processed);

            cv::imshow("minimap", temp_map_);
            cv::waitKey(1);
        }

        for (size_t i = 0; i < enemy_positions_processed.size(); i++) {
            // 串口数据打包
            uint8_t serial_data[19];
            serial_data_pack(serial_data, enemy_positions_processed[i], i);
            serial_util::dji_crc::append_crc16(serial_data);

            // 串口数据发送
            radar_serial_->write(serial_data, sizeof(serial_data));
        }
    }
    void receivev_enemy_data(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        try {
            enemy_positions_.clear();
            logger_.INFO("-->Received data from car_detect(size:" + std::to_string(msg->data.size()) + ")");

            // 串口数据解析
            if (msg->data.size() != 0) {
                for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
                    map_robot_data_t enemy_robot_position;

                    enemy_robot_position.target_robot_id = msg->data.data()[i];
                    if (radar_serial_config_.friend_side == BLUE) {
                        enemy_robot_position.target_position_x = msg->data.data()[i + 1];
                        enemy_robot_position.target_position_y = msg->data.data()[i + 2];
                    } else {
                        enemy_robot_position.target_position_x = 28 - msg->data.data()[i + 1];
                        enemy_robot_position.target_position_y = 15 - msg->data.data()[i + 2];
                    }

                    enemy_positions_.push_back(enemy_robot_position);
                }
            }

        } catch (const std::exception& e) {
            logger_.ERRORS("[x]Error in command_callback: " + std::string(e.what()));
        }
    }
};

/*  数据订阅&串口发送类
 *  订阅数据并发送到串口
 */
class MainProcess : public rclcpp::Node {
public:
    // 构造函数
    MainProcess(std::string name)
        : Node(name)
    {
        radar_serial_handler_ = std::make_unique<Serial_handler>();
        position_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "car_positions", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                radar_serial_handler_->receivev_enemy_data(msg);
            });
        using namespace std::chrono_literals;
        serial_handle_timer_ = this->create_wall_timer(10ms, [this]() { radar_serial_handler_->send_radar_cmd(); });

        if (radar_serial_handler_->debug_) {
            cv::namedWindow("minimap", 0);
            cv::resizeWindow("minimap", cv::Size(960, 540));
        }
        radar_serial_handler_->logger_.INFO("[√]Node initialized.");
        radar_serial_handler_->logger_.INFO("Friend Side:(RED 100 |BLUE 101) " + std::to_string(radar_serial_handler_->radar_serial_config_.friend_side));
        radar_serial_handler_->logger_.INFO("DEBUG: " + std::to_string(radar_serial_handler_->radar_serial_config_.debug));
    }

private:
    std::unique_ptr<Serial_handler> radar_serial_handler_;
    // 声明订阅者
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>> position_subscription_;
    std::shared_ptr<rclcpp::TimerBase> serial_handle_timer_;
};
}