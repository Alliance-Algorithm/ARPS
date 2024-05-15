/*
 *@email lisiyao20041017@gmail.com
 *最后更新时间:2024/3/19 22p.m.
 *更新人:算法组-Lisiyao
 *更新内容:添加了日志系统
 */
#include "../include/Logger.hpp"
#include "../include/nlohmann/json.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <charconv>
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
#include <serial/serial.h>
#include <serial_util/crc/dji_crc.hpp>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#define RED 100
#define BLUE 101

using json = nlohmann::json;

// 读取配置文件
struct Configs {
    bool debug;
    int Friend_Side;
    std::string log_path;
    std::string serial_port;
};

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

Configs configs = read_config("./resources/config.json");

bool debug = configs.debug;

std::map<float, std::string> categories = { { 0, "CantIdentfy" }, { 1, "R_Hero" }, { 2, "R_Engineer" },
    { 3, "R_Solider_3" }, { 4, "R_Solider_4" }, { 5, "R_Solider_5" },
    { 7, "R_Sentry" }, { 101, "B_Hero" }, { 102, "B_Engineer" },
    { 103, "B_Solider_3" }, { 104, "B_Solider_4" }, { 105, "B_Solider_5" },
    { 107, "B_Sentry" } };

/*
 * 串口数据格式
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

// 日志类初始化
Logger logger(Logger::file, Logger::debug, configs.log_path);

cv::Mat minimap_image = cv::imread("./resources/images/RM2024Battlefield.png");
cv::Mat temp_map;

// 串口初始化
serial::Serial radar_serial = serial::Serial(configs.serial_port, 115200U, serial::Timeout::simpleTimeout(50U), serial::eightbits, serial::parity_none,
    serial::stopbits_one, serial::flowcontrol_none);

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

/*  数据订阅&串口发送类
 *  订阅数据并发送到串口
 */
class PositionsSubscriber : public rclcpp::Node {
public:
    // 构造函数
    PositionsSubscriber(std::string name)
        : Node(name)
    {
        position_subscribe_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "car_positions", 10, std::bind(&PositionsSubscriber::command_callback, this, std::placeholders::_1));
    }

private:
    // 声明订阅者
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_subscribe_;
    // 收到数据的回调函数
    void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        try {
            logger.INFO("-->Received data from car_detect(size:" + std::to_string(msg->data.size()) + ")");
            // 串口数据初始化容器
            std::vector<map_robot_data_t> emeny_robot_positions;

            if (debug)
                minimap_image.copyTo(temp_map);

            // 串口数据解析
            if (msg->data.size() != 0) {
                for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
                    map_robot_data_t emeny_robot_position;

                    emeny_robot_position.target_robot_id = msg->data.data()[i];

                    if (configs.Friend_Side == BLUE) {
                        emeny_robot_position.target_position_x = msg->data.data()[i + 1];
                        emeny_robot_position.target_position_y = msg->data.data()[i + 2];
                    } else {
                        emeny_robot_position.target_position_x = 28 - msg->data.data()[i + 1];
                        emeny_robot_position.target_position_y = 15 - msg->data.data()[i + 2];
                    }

                    if (debug) {
                        int car_position_x = int((emeny_robot_position.target_position_x / 28.0) * 1854.0);
                        int car_position_y = int(((15.0 - emeny_robot_position.target_position_y) / 15.0) * 996.0);
                        cv::Point2d car_point(car_position_x, car_position_y);
                        cv::Point2d id_point(car_position_x + 30, car_position_y + 10);

                        cv::Scalar color = int(emeny_robot_position.target_robot_id) > 7 ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255);

                        cv::circle(temp_map, car_point, 20, color, -1);
                        cv::putText(temp_map, categories[int(emeny_robot_position.target_robot_id)], id_point, 1, 2,
                            cv::Scalar(255, 255, 255), 2);
                    }

                    emeny_robot_positions.push_back(emeny_robot_position);
                }
            }

            if (debug) {
                cv::imshow("minimap", temp_map);
                cv::waitKey(1);
            }

            for (size_t i = 0; i < emeny_robot_positions.size(); i++) {
                // 串口数据打包
                uint8_t serial_data[19];
                serial_data_pack(serial_data, emeny_robot_positions[i], i);
                serial_util::dji_crc::append_crc16(serial_data);
                // 串口数据发送
                radar_serial.write(serial_data, sizeof(serial_data));
            }
            logger.INFO("Send data to radar_serial(size:" + std::to_string(emeny_robot_positions.size()) + ")");
        } catch (const std::exception& e) {
            logger.ERRORS("[x]Error in command_callback: " + std::string(e.what()));
        }
    }
};

// 串口发送数据
int main(int argc, char** argv)
{
    if (debug) {
        cv::namedWindow("minimap", 0);
        cv::resizeWindow("minimap", cv::Size(960, 540));
    }

    try {
        // 初始化节点
        logger.INFO("Initializing node...");
        rclcpp::init(argc, argv);

        // [创建对应节点的共享指针对象]
        auto positions_subscriber = std::make_shared<PositionsSubscriber>("positions_subscriber");
        logger.INFO("[√]Node initialized.");

        logger.INFO("Friend Side:(RED 100 |BLUE 101) " + std::to_string(configs.Friend_Side));
        logger.INFO("DEBUG: " + std::to_string(configs.debug));

        // [运行节点，并检测退出信号]
        rclcpp::spin(positions_subscriber);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        logger.ERRORS("[x]Error in command_callback: " + std::string(e.what()));
    }

    return 0;
}