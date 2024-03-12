#include <iostream>
#include <serial/serial.h>

int main()
{
    serial::Serial radar_serial = serial::Serial();
    std::cout << "success\n";
    return 0;
}