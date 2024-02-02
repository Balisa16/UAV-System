#pragma once

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <enum.hpp>

namespace EMIRO
{
    class UART
    {
    public:
        UART();

        void init(std::string device = "/dev/ttyTHS1", speed_t baudrate = B9600);

        // Receive
        std::string read_s(size_t length = 1024);

        bool read_pose(Position *pos, Quaternion *quat);

        // Send
        void write_s(std::string data = "None");

        void write_pose(Position *pos, Quaternion *quat);

        ~UART();

    private:
        int uart_fd;
        std::string device;
        speed_t baudrate;
    };
}