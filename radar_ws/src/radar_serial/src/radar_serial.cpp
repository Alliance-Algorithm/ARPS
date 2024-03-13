#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

/*
 * 串口数据格式
 * 帧头(4字节) + cmd id(2字节) + 数据(10字节) + CRC16(2字节)
 * 帧头: SOF(1字节) + 数据长度(2字节) + 包序号(1字节) + CRC8(1字节)
 * 数据: 目标机器人ID(2字节) + 目标机器人x坐标(4字节) + 目标机器人y坐标(4字节)
 * 帧尾: CRC16(2字节)
 */
using map_robot_data_t = struct
{
    uint16_t target_robot_id;
    uint32_t target_position_x;
    uint32_t target_position_y;
};

/*
 * CRC16校验
 * @param data 数据
 * @param len 数据长度
 * @return CRC16校验值
 */
uint16_t CRC16_Check(const uint8_t *data, uint8_t len)
{
    uint16_t CRC16 = 0xFFFF;
    uint8_t state, i, j;
    for (i = 0; i < len; i++)
    {
        CRC16 ^= data[i];
        for (j = 0; j < 8; j++)
        {
            state = CRC16 & 0x01;
            CRC16 >>= 1;
            if (state)
            {
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
uint8_t CRC8_Check(const uint8_t *data, uint8_t len)
{
    uint8_t CRC8 = 0;
    uint8_t i, j;
    for (i = 0; i < len; i++)
    {
        CRC8 ^= data[i];
        for (j = 0; j < 8; j++)
        {
            if (CRC8 & 0x80)
            {
                CRC8 = (CRC8 << 1) ^ 0x07;
            }
            else
            {
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
void serial_data_pack(uint8_t *serial_data, map_robot_data_t emeny_robot_positions, int req)
{
    // 串口数据
    uint8_t serial_data[19];
    // 帧头
    uint8_t frame_header[] = {0xA5, 0x0A >> 8, 0x0A, (uint8_t)req}; // SOF, 数据长度高8位, 数据长度低8位, 包序号
    uint8_t len = sizeof(frame_header) / sizeof(frame_header[0]);   // 帧头长度
    uint8_t frame_header_CRC8 = CRC8_Check(frame_header, len);      // 帧头CRC8校验
    frame_header[len] = frame_header_CRC8;

    serial_data[0] = frame_header[0];
    serial_data[1] = frame_header[1];
    serial_data[2] = frame_header[2];
    serial_data[3] = frame_header[3];
    serial_data[4] = frame_header[4];
    serial_data[5] = 0x0305 >> 8; // cmd id高8位
    serial_data[6] = 0x0305;      // cmd id低8位

    // 打包数据
    serial_data[7] = emeny_robot_positions.target_robot_id >> 8;
    serial_data[8] = emeny_robot_positions.target_robot_id;
    serial_data[9] = emeny_robot_positions.target_position_x >> 24;
    serial_data[10] = emeny_robot_positions.target_position_x >> 16;
    serial_data[11] = emeny_robot_positions.target_position_x >> 8;
    serial_data[12] = emeny_robot_positions.target_position_x;
    serial_data[13] = emeny_robot_positions.target_position_y >> 24;
    serial_data[14] = emeny_robot_positions.target_position_y >> 16;
    serial_data[15] = emeny_robot_positions.target_position_y >> 8;
    serial_data[16] = emeny_robot_positions.target_position_y;

    // CRC16校验
    uint16_t CRC16 = CRC16_Check(serial_data + 5, 11);
    serial_data[17] = CRC16 >> 8;
    serial_data[18] = CRC16;
}

// 串口发送数据
int main()
{
    serial::Serial radar_serial = serial::Serial("/dev/ttyUSB0",
                                                 115200U,
                                                 serial::Timeout(),
                                                 serial::eightbits,
                                                 serial::parity_none,
                                                 serial::stopbits_one,
                                                 serial::flowcontrol_none);
    map_robot_data_t emeny_robot_positions;
    uint8_t serial_data[19];
    emeny_robot_positions.target_position_x = 11.0;
    emeny_robot_positions.target_position_y = 22.0;
    emeny_robot_positions.target_robot_id = 7;

    serial_data_pack(serial_data, emeny_robot_positions, 1);
    int count = 1;
    while (true)
    {
        radar_serial.write(serial_data, sizeof(serial_data));
        std::cout << count++ << " Send radar data" << std::endl;
        sleep(1);
    }

    return 0;
}