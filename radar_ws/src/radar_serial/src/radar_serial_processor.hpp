#pragma once

#include "Logger.hpp"
#include "nlohmann/json.hpp"
#include "radar_cmd.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include <rcl/event.h>
#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <string>
#include <unistd.h>
#include <vector>

using json = nlohmann::json;

namespace radar {
// 读取配置文件
struct Configs {
    bool debug;
    int Friend_Side;
    std::string log_path;
    std::string serial_port;
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

class Radar_serial_handler {
public:
    Configs radar_serial_config_;
    std::map<float, std::string> categories_;
    bool debug_;
    Logger logger_;
    cv::Mat minimap_image_;
    cv::Mat temp_map_;

    std::shared_ptr<serial::Serial> radar_serial_;
    radar::Radar_decisioner radar_decisioner_;

public:
    Radar_serial_handler()
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
            usleep(100000);
        }

        radar_decisioner_ = radar::Radar_decisioner(radar_serial_, &logger_, radar_serial_config_.Friend_Side);
    }

    Configs read_config(std::string filename)
    {
        std::ifstream i(filename);
        json config_json;
        i >> config_json;

        Configs config;

        config.debug = config_json["SERIAL_DEBUG"].get<std::string>() == "true" ? true : false;
        config.Friend_Side = config_json["FRIEND_SIDE"].get<std::string>() == "RED" ? RED : BLUE;
        config.log_path = config_json["serial_log_path"].get<std::string>();
        config.serial_port = config_json["serial_port"].get<std::string>();
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

        serial_data[0] = frame_header[0];
        serial_data[1] = frame_header[1];
        serial_data[2] = frame_header[2];
        serial_data[3] = frame_header[3];
        serial_data[4] = frame_header[4];
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
    void debug_imageshow(map_robot_data_t emeny_robot_position)
    {
        int car_position_x = int((emeny_robot_position.target_position_x / 28.0) * 1854.0);
        int car_position_y = int(((15.0 - emeny_robot_position.target_position_y) / 15.0) * 996.0);
        cv::Point2d car_point(car_position_x, car_position_y);
        cv::Point2d id_point(car_position_x + 30, car_position_y + 10);

        cv::Scalar color = int(emeny_robot_position.target_robot_id) > 7 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

        cv::circle(temp_map_, car_point, 20, color, -1);
        cv::putText(temp_map_, categories_[int(emeny_robot_position.target_robot_id)], id_point, 1, 2,
            cv::Scalar(255, 255, 255), 2);
    }

public:
    void send_radar_cmd()
    {
        radar_decisioner_.receive_status_data();
        uint8_t radar_decision_data[10];
        serial_data_pack(radar_decision_data, radar_decisioner_.radar_information_.double_debuff_cmd, 0);
        serial_util::dji_crc::append_crc16(radar_decision_data);
        radar_serial_->write(radar_decision_data, sizeof(radar_decision_data));
    }
    void resend_data(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        try {
            logger_.INFO("-->Received data from car_detect(size:" + std::to_string(msg->data.size()) + ")");
            // 串口数据初始化容器
            std::vector<map_robot_data_t> emeny_robot_positions;

            if (debug_)
                minimap_image_.copyTo(temp_map_);

            // 串口数据解析
            if (msg->data.size() != 0) {
                for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
                    map_robot_data_t emeny_robot_position;

                    emeny_robot_position.target_robot_id = msg->data.data()[i];

                    if (radar_serial_config_.Friend_Side == BLUE) {
                        emeny_robot_position.target_position_x = msg->data.data()[i + 1];
                        emeny_robot_position.target_position_y = msg->data.data()[i + 2];
                    } else {
                        emeny_robot_position.target_position_x = 28 - msg->data.data()[i + 1];
                        emeny_robot_position.target_position_y = 15 - msg->data.data()[i + 2];
                    }

                    if (debug_)
                        debug_imageshow(emeny_robot_position);

                    emeny_robot_positions.push_back(emeny_robot_position);
                }
            }

            if (debug_) {
                cv::imshow("minimap", temp_map_);
                cv::waitKey(1);
            }

            for (size_t i = 0; i < emeny_robot_positions.size(); i++) {
                // 串口数据打包
                uint8_t serial_data[19];
                serial_data_pack(serial_data, emeny_robot_positions[i], i);
                serial_util::dji_crc::append_crc16(serial_data);

                // 串口数据发送
                radar_serial_->write(serial_data, sizeof(serial_data));
            }
            logger_.INFO("Send data to radar_serial(size:" + std::to_string(emeny_robot_positions.size()) + ")");
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
        radar_serial_handler_ = std::make_unique<Radar_serial_handler>();
        position_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "car_positions", 10, [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                radar_serial_handler_->resend_data(msg);
            });
        using namespace std::chrono_literals;
        serial_handle_timer_ = this->create_wall_timer(2ms, [this]() { radar_serial_handler_->send_radar_cmd(); });

        if (radar_serial_handler_->debug_) {
            cv::namedWindow("minimap", 0);
            cv::resizeWindow("minimap", cv::Size(960, 540));
        }
        radar_serial_handler_->logger_.INFO("[√]Node initialized.");
        radar_serial_handler_->logger_.INFO("Friend Side:(RED 100 |BLUE 101) " + std::to_string(radar_serial_handler_->radar_serial_config_.Friend_Side));
        radar_serial_handler_->logger_.INFO("DEBUG: " + std::to_string(radar_serial_handler_->radar_serial_config_.debug));
    }

private:
    std::unique_ptr<Radar_serial_handler> radar_serial_handler_;
    // 声明订阅者
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>> position_subscription_;
    std::shared_ptr<rclcpp::TimerBase> serial_handle_timer_;
};
}