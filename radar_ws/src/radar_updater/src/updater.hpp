#pragma once

// custom headers
#include "include/nlohmann/json.hpp"
#include "include/radar_informations.hpp"
#include "include/received_packages.hpp"

// denpendent headers
#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <serial_util/package_receive.hpp>

// std headers
#include <bitset>
#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <unistd.h>
#include <utility>
#include <vector>

// OpenCV headers
#include <opencv2/opencv.hpp>

// ROS2 headers
#include <rcl/event.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rcutils/types/rcutils_ret.h>
#include <std_msgs/msg/float32_multi_array.hpp>

// define macros
#define RED 100
#define BLUE 101

namespace radar {

class Updater : public rclcpp::Node {
public:
    Updater();
    ~Updater();

    std::shared_ptr<info::Informations> radar_information_;
    info::Configs radar_config_;

    size_t cache_size_ = 0;

    cv::Mat minimap_image_;
    cv::Mat temp_img_;
    std::map<float, std::string> labels_;
    package::receive::Frame frame_;

    std::shared_ptr<serial::Serial> serial_;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>> position_subscription_;
    std::shared_ptr<rclcpp::TimerBase> updater_timer;

public:
    /* main processor */
    void update_radar_status();
    void process_data();

private:
    /* initializition */
    info::Configs read_config(std::string filename);
    void initialize_serial();
    void initialize_processors();
    void initialize_callbacks();
    void initialize_debug_plotter();

private:
    /* receiver */
    void receive_topic_data(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void receive_referee_data();

    /* receivered data processor */
    void process_frame_info();
    void update_game_status();
    void update_sentry_hp();
    void update_buff_info();
    void update_radar_info();
    void update_enemy_status_by_sentry();

private:
    /* submitter */
    void submit_serial_data();
    void enemy_position_data_pack(uint8_t* serial_data, std::map<int, info::enemy_robot_position> enemy_robot_positions, int req);
    void enemy_position_data_to_sentry_pack(uint8_t* serial_data, std::map<int, info::enemy_robot_position> enemy_robot_positions, int req);
    void radar_cmd_data_pack(uint8_t* serial_data, int double_debuff_cmd, int req);

private:
    /* util functions */
    void plot();
};
}