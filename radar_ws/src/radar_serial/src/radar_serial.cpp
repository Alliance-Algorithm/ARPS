/*
 *@email lisiyao20041017@gmail.com
 *最后更新时间:2024/3/19 22p.m.
 *更新人:算法组-Lisiyao
 *更新内容:添加了日志系统
 */
#include "Logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <cstdint>
#include <cstring>
#include <iostream>
#include <map>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <serial/serial.h>
#include <unistd.h>
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <vector>

#define DEBUG true

std::map<float, std::string> categories =
			{{0,"CantIdentfy"}, 
			{1,"R_Hero"},{2,"R_Engineer"}, {3,"R_Solider_1"}, {4,"R_Solider_2"}, {5,"R_Solider_3"}, {7,"R_Sentry"},
			{101,"B_Hero"},{102,"B_Engineer"}, {103,"B_Solider_1"}, {104,"B_Solider_2"},{105,"B_Solider_3"} ,{107,"B_Sentry"}};

/*
 * 串口数据格式
 * 帧头(5字节) + cmd id(2字节) + 数据(10字节) + CRC16(2字节)
 * 帧头: SOF(1字节) + 数据长度(2字节) + 包序号(1字节) + CRC8(1字节)
 * 数据: 目标机器人ID(2字节) + 目标机器人x坐标(4字节) + 目标机器人y坐标(4字节)
 * 帧尾: CRC16(2字节)
 */
using map_robot_data_t = struct {
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
};

Logger logger(Logger::file, Logger::debug, "./resources/user_logs/serial.log");

#if DEBUG
cv::Mat minimap_image = cv::imread("./resources/images/RM2024Battlefield.png");
cv::Mat temp_map;
#endif

// 串口初始化
serial::Serial radar_serial =
		serial::Serial("/dev/ttyUSB1", 115200U, serial::Timeout::simpleTimeout(50U),
									 serial::eightbits, serial::parity_none, serial::stopbits_one,
									 serial::flowcontrol_none);
/*
 * CRC16校验
 * @param data 数据
 * @param len 数据长度
 * @return CRC16校验值
 */
uint16_t CRC16_Check(const uint8_t *data, uint8_t len) {
	uint16_t CRC16 = 0xFFFF;
	uint8_t state, i, j;
	for (i = 0; i < len; i++) {
		CRC16 ^= data[i];
		for (j = 0; j < 8; j++) {
			state = CRC16 & 0x01;
			CRC16 >>= 1;
			if (state) {
				CRC16 ^= 0xA001;
			}
		}
	}
	return CRC16;
}

/*
 * CRC8校验
 * @param data 数据
 * @param len 数据长度
 * @return CRC8校验值
 */
uint8_t CRC8_Check(const uint8_t *data, uint8_t len) {
	uint8_t CRC8 = 0;
	uint8_t i, j;
	for (i = 0; i < len; i++) {
		CRC8 ^= data[i];
		for (j = 0; j < 8; j++) {
			if (CRC8 & 0x80) {
				CRC8 = (CRC8 << 1) ^ 0x07;
			} else {
				CRC8 <<= 1;
			}
		}
	}
	return CRC8;
}

/*
 * 串口数据打包
 * @param emeny_robot_positions 敌方机器人位置信息
 * @param req 包序号
 * @return serial_data 串口数据
 */
void serial_data_pack(uint8_t *serial_data,
											map_robot_data_t emeny_robot_positions, int req) {
	// 帧头
	uint8_t frame_header[5] = {
			0xA5, 0x0A >> 8, 0x0A,
			(uint8_t)req}; // SOF, 数据长度高8位, 数据长度低8位, 包序号
	uint8_t len = sizeof(frame_header) / sizeof(frame_header[0]); // 帧头长度
	uint8_t frame_header_CRC8 = CRC8_Check(frame_header, len); // 帧头CRC8校验
	frame_header[len - 1] = frame_header_CRC8;

	serial_data[0] = frame_header[0];
	serial_data[1] = frame_header[1];
	serial_data[2] = frame_header[2];
	serial_data[3] = frame_header[3];
	serial_data[4] = frame_header[4];
	serial_data[5] = 0x0305 >> 8;   // cmd id高8位
	serial_data[6] = 0x0305 & 0xFF; // cmd id低8位

	// 打包数据
	memccpy(&serial_data[7],&emeny_robot_positions.target_robot_id, 1, 2);
	memccpy(&serial_data[9],&emeny_robot_positions.target_position_x, 1, 4);
	memccpy(&serial_data[13],&emeny_robot_positions.target_position_y, 1, 4);

	// CRC16校验
	uint16_t CRC16 = CRC16_Check(serial_data + 5, 11);
	serial_data[17] = CRC16 >> 8;
	serial_data[18] = CRC16;
}

/*  数据订阅&串口发送类
 *  订阅数据并发送到串口
 */
class PositionsSubscriber : public rclcpp::Node {
public:
	// 构造函数
	PositionsSubscriber(std::string name) : Node(name) {
		position_subscribe_ =
				this->create_subscription<std_msgs::msg::Float32MultiArray>(
						"car_positions", 10,
						std::bind(&PositionsSubscriber::command_callback, this,
											std::placeholders::_1));
	}

private:
	// 声明订阅者
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
			position_subscribe_;
	// 收到数据的回调函数
	void command_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
		try {
			logger.INFO("-->Received data from car_detect(size:" +
									std::to_string(msg->data.size()) + ")");
			// 串口数据初始化容器
			std::vector<map_robot_data_t> emeny_robot_positions;


			#if DEBUG
			minimap_image.copyTo(temp_map);
			#endif

			// 串口数据解析
			if (msg->data.size() != 0) {
				for (size_t i = 0; i < msg->data.size() - 2; i += 3) {
					map_robot_data_t emeny_robot_position;

					emeny_robot_position.target_robot_id = msg->data.data()[i];
					emeny_robot_position.target_position_x = msg->data.data()[i + 1];
					emeny_robot_position.target_position_y = msg->data.data()[i + 2];

					#if DEBUG
					int car_position_x = int((emeny_robot_position.target_position_x / 28.0) * 1854.0);
					int car_position_y = int(((15.0 - emeny_robot_position.target_position_y) / 15.0) * 996.0);
					cv::Point2d car_point(car_position_x,car_position_y);
					cv::Point2d id_point(car_position_x + 30,car_position_y + 10);

					cv::Scalar color = int(emeny_robot_position.target_robot_id) > 7 ? cv::Scalar(255,0,0) : cv::Scalar(0,0,255);

					cv::circle(temp_map, car_point,20, color,-1);
					cv::putText(temp_map, categories[int(emeny_robot_position.target_robot_id)], id_point,1, 2, cv::Scalar(255,255,255),2);
					#endif

					emeny_robot_positions.push_back(emeny_robot_position);
				}
			}

			#if DEBUG
			cv::imshow("minimap",temp_map);
			cv::waitKey(1);
			#endif

			for (size_t i = 0; i < emeny_robot_positions.size(); i++) {
				// 串口数据打包
				uint8_t serial_data[19];
				serial_data_pack(serial_data, emeny_robot_positions[i], i);

				// 串口数据发送
				radar_serial.write(serial_data, sizeof(serial_data));
			}
			logger.INFO("Send data to radar_serial(size:" +
									std::to_string(emeny_robot_positions.size()) + ")");
		} catch (const std::exception &e) {
			logger.ERRORS("[x]Error in command_callback: " + std::string(e.what()));
		}
	}
};

// 串口发送数据
int main(int argc, char **argv) {
	cv::namedWindow("minimap",0);
	cv::resizeWindow("minimap",cv::Size(960,540));

	try {
		// 初始化节点
		logger.INFO("Initializing node...");
		rclcpp::init(argc, argv);

		// [创建对应节点的共享指针对象]
		auto positions_subscriber =
				std::make_shared<PositionsSubscriber>("positions_subscriber");
		logger.INFO("[√]Node initialized.");

		// [运行节点，并检测退出信号]
		rclcpp::spin(positions_subscriber);
		rclcpp::shutdown();
	} catch (const std::exception &e) {
		logger.ERRORS("[x]Error in command_callback: " + std::string(e.what()));
	}

	return 0;
}