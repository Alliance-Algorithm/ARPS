#include "../updater.hpp"
#include <cstdint>
#include <rclcpp/logging.hpp>

namespace radar {
/*
 * 提交雷达数据
 */
void Updater::submit_serial_data()
{
    if (!serial_->isOpen())
        return;

    /* -- Enemy position datas --
     * cmd_id 0x0305
     * package_length 33
     * data_length 24
     * frenquency 5Hz
     */
    uint8_t enemy_position_data[33];
    enemy_position_data_pack(enemy_position_data, radar_information_->enemy_robot_positions, 0);
    serial_util::dji_crc::append_crc16(enemy_position_data);
    serial_->write(enemy_position_data, sizeof(enemy_position_data));
    RCLCPP_INFO(get_logger(), "* Radar data submitted [0x0305] | Size: %lu", radar_information_->enemy_robot_positions.size());

    /* -- Enemy position datas to sentry --
     * cmd_id 0x0301
     * sub_id 0x0222
     * package_length 63
     * data_length 54
     * frenquency 5Hz
     */
    uint8_t enemy_position_data_to_sentry[63];
    enemy_position_data_to_sentry_pack(enemy_position_data_to_sentry, radar_information_->enemy_robot_positions, 0);
    serial_util::dji_crc::append_crc16(enemy_position_data_to_sentry);
    serial_->write(enemy_position_data_to_sentry, sizeof(enemy_position_data_to_sentry));

    /* -- Radar decision datas --
     * cmd_id 0x0301
     * sub_id 0x0121
     * package_length 16
     * data_length 7
     * frenquency 5Hz
     */
    uint8_t radar_decision_data[16];
    radar_cmd_data_pack(radar_decision_data, radar_information_->double_debuff_cmd_, 0);
    serial_util::dji_crc::append_crc16(radar_decision_data);
    serial_->write(radar_decision_data, sizeof(radar_decision_data));
    RCLCPP_INFO(get_logger(), "* Radar cmd submitted [0x0301|0x0121] | CMD: %d", radar_information_->double_debuff_cmd_);
}

/*
 * 串口数据打包(新)
 * @param enemy_robot_positions 敌方机器人位置信息
 * @param req 包序号
 * @return serial_data 串口数据
 */
void Updater::enemy_position_data_pack(uint8_t* serial_data, std::map<int, info::enemy_robot_position> enemy_robot_positions, int req)
{
    std::vector<cv::Point2d> enemy_positions(6, cv::Point2d(0, 0));
    for (int i = 0; i < 6; i++) {
        auto it = enemy_robot_positions.find(radar_information_->enemy_catorgories[i]);
        if (it != enemy_robot_positions.end()) {
            // m to cm
            enemy_positions[i] = cv::Point2d(it->second.x * 100, it->second.y * 100);
        }
    }

    info::enemy_robot_position_new enemy_robot_position_to_send = info::enemy_robot_position_new {
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
        = { 0xA5, static_cast<uint8_t>(24), 0, static_cast<uint8_t>(req) }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
    serial_util::dji_crc::append_crc8(frame_header);

    for (int i = 0; i < 5; i++)
        serial_data[i] = frame_header[i];
    reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0305;

    reinterpret_cast<info::enemy_robot_position_new&>(serial_data[7]) = enemy_robot_position_to_send;
}

/*
 * 串口数据打包
 * @param double_debuff_cmd 雷达决策命令
 * @param req 包序号
 * @return serial_data 串口数据
 */
void Updater::radar_cmd_data_pack(uint8_t* serial_data, int double_debuff_cmd, int req)
{
    // 帧头
    // TODO: 数据长度
    uint8_t frame_header[5] = { 0xA5, 0x07, 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
    serial_util::dji_crc::append_crc8(frame_header);

    for (int i = 0; i < 5; i++)
        serial_data[i] = frame_header[i];

    reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0301; // 命令id
    reinterpret_cast<uint16_t&>(serial_data[7]) = 0x0121; // 子内容id
    reinterpret_cast<uint16_t&>(serial_data[9]) = radar_config_.friend_side == RED ? 9 : 109; // 雷达id
    reinterpret_cast<uint16_t&>(serial_data[11]) = 0x8080; // 裁判系统id
    reinterpret_cast<int&>(serial_data[13]) = double_debuff_cmd;
}

/*
 * 给哨兵的串口数据打包
 * @param enemy_robot_positions 敌方机器人位置信息
 * @param req 包序号
 * @return serial_data 串口数据
 */
void Updater::enemy_position_data_to_sentry_pack(uint8_t* serial_data, std::map<int, info::enemy_robot_position> enemy_robot_positions, int req)
{
    // 帧头
    uint8_t frame_header[5]
        = { 0xA5, static_cast<uint8_t>(54), 0, (uint8_t)req }; // SOF, 数据长度高8位, 数据长度低8位, 包序号
    serial_util::dji_crc::append_crc8(frame_header);

    for (int i = 0; i < 5; i++)
        serial_data[i] = frame_header[i];
    reinterpret_cast<uint16_t&>(serial_data[5]) = 0x0301;

    // 与哨兵的约定id : 0x0222
    reinterpret_cast<uint16_t&>(serial_data[7]) = 0x0222;
    reinterpret_cast<uint16_t&>(serial_data[9]) = radar_config_.friend_side == RED ? 9 : 109;
    reinterpret_cast<uint16_t&>(serial_data[11]) = radar_config_.friend_side == RED ? 7 : 107;

    for (int i = 0; i < 6; i++) {
        info::position_data_with_sentry single_enemy_position;

        auto it = enemy_robot_positions.find(radar_information_->enemy_catorgories[i]);
        if (it != enemy_robot_positions.end()) {
            single_enemy_position.positions[i].x = it->second.x;
            single_enemy_position.positions[i].y = it->second.y;
        } else {
            single_enemy_position.positions[i].x = -114514;
            single_enemy_position.positions[i].y = -114514;
        }
        reinterpret_cast<info::position_data_with_sentry&>(serial_data[13]) = single_enemy_position;
    }
}
}