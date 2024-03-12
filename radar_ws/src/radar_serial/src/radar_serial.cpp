#include <iostream>
#include <serial/serial.h>
#include <unistd.h>

typedef struct _packed
{
    uint16_t target_robot_id;
    float target_position_x;
    float target_position_y;
} map_robot_data_t;

int main()
{
    serial::Serial radar_serial = serial::Serial("/dev/ttyUSB0",
                                                 115200U,
                                                 serial::Timeout(),
                                                 serial::eightbits,
                                                 serial::parity_none,
                                                 serial::stopbits_one,
                                                 serial::flowcontrol_none);
    map_robot_data_t *emeny_robot_positions;
    emeny_robot_positions = (map_robot_data_t *)malloc(sizeof(map_robot_data_t));
    emeny_robot_positions->target_position_x = 1.0;
    emeny_robot_positions->target_position_y = 2.0;
    emeny_robot_positions->target_robot_id = 1;

    while (true)
    {
        radar_serial.write((uint8_t *)emeny_robot_positions, sizeof(emeny_robot_positions));
        std::cout << "Send radar data" << std::endl;
        sleep(1);
    }

    return 0;
}