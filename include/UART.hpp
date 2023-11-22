#ifndef UART_HEADER
#define UART_HEADER

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <sstream>

namespace EMIRO
{
#ifndef POSITION_HEADER
#define  POSITION_HEADER
    typedef struct{
        float x, y, z;
    }Position;
#endif

#ifndef QUATERNION_HEADER
#define QUATERNION_HEADER
    typedef struct{
        float w, x, y, z;
    }Quaternion;
#endif

    class UART
    {
    private:
        int uart_fd;
        std::string device;
        speed_t baudrate;
    public:
        UART();
        void init(std::string device = "/dev/ttyTHS1", speed_t baudrate = B9600);

        // Receive
        std::string read_s(size_t length = 1024);
        bool read_pose(Position* pos, Quaternion* quat);

        // Send
        void write_s(std::string data = "None");
        void write_pose(Position* pos, Quaternion* quat);
        ~UART();
    };
}

#endif
