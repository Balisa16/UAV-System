#include <UART.hpp>

namespace EMIRO
{
    UART::UART()
    {

    }

    void UART::init(std::string device, speed_t baudrate)
    {
        this->device = device;
        this->baudrate = baudrate;

        uart_fd = open(this->device.c_str(), O_RDWR);
        if (uart_fd == -1) {
            perror("Error opening UART device");
            exit(EXIT_FAILURE);
        }

        // Configure UART settings
        struct termios uart_config;
        if (tcgetattr(uart_fd, &uart_config) == -1) {
            perror("Error getting UART configuration");
            close(uart_fd);
            exit(EXIT_FAILURE);
        }

        uart_config.c_cflag &= ~PARENB; // No parity bit
        uart_config.c_cflag &= ~CSTOPB; // 1 stop bit
        uart_config.c_cflag &= ~CSIZE;
        uart_config.c_cflag |= CS8; // 8 data bits
        uart_config.c_cflag &= ~CRTSCTS; // No hardware flow control
        uart_config.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control lines

        uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
        uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable special handling of received bytes

        // tty.c_oflag &= ~OPOST; // Raw output mode
        uart_config.c_oflag &= ~OPOST; // Raw output mode

        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // Disable canonical mode, disable signal handling
        uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN); // Disable canonical mode, disable signal handling

        cfsetospeed(&uart_config, this->baudrate);
        cfsetispeed(&uart_config, this->baudrate);


        // Apply the configuration
        if (tcsetattr(uart_fd, TCSANOW, &uart_config) == -1) {
            perror("Error setting UART configuration");
            close(uart_fd);
            exit(EXIT_FAILURE);
        }
    }

    std::string UART::read_s(size_t length)
    {
        char buffer[length];
        while (true) {
            if (read(uart_fd, buffer, length) > 0)
            {
                int buf_len = strlen(buffer);
                char new_c[buf_len-1];
                for (int i = 0; i < buf_len-1; ++i)
                    new_c[i] = buffer[i];

                /*std::cout << "Len " << buf_len << std::endl;
                for (int i = 0; i <= 3; ++i)
                    buffer[buf_len - i] = 'a';
                return buffer;*/
                return new_c;
            }
        }
    }

    bool UART::read_pose(Position* pos, Quaternion* quat)
    {
        // Read data
        char buffer[1024];
        while (true) {
            if (read(uart_fd, buffer, 1024) > 0)
                break;
        }

        // Parse data
        std::istringstream iss(buffer);
        std::string token;

        int cnt = 0;

        try{
            while (std::getline(iss, token, ',')) {
                float val = std::stof(token);
                switch (cnt)
                {
                case 0:
                    pos->x = val;
                    break;
                case 1:
                    pos->y = val;
                    break;
                case 2:
                    pos->z = val;
                    break;
                case 3:
                    quat->w = val;
                    break;
                case 4:
                    quat->x = val;
                    break;
                case 5:
                    quat->y = val;
                    break;
                case 6:
                    quat->z = val;
                    break;
                
                default:
                    break;
                }
                cnt++;
            }
        }catch(std::exception& ex) {return false;}

        if(cnt < 6)
		std::cerr << "Invalid data. Found " << cnt << " data\n";
	else
		return true;
	return false;
    }

    void UART::write_s(std::string data)
    {
        const char* data_char = data.c_str();
        int len = data.length();

        ssize_t bytes_written = write(uart_fd, data_char, len-1);

        if (bytes_written == -1) {
            perror("Error writing to UART");
            close(uart_fd);
            exit(EXIT_FAILURE);
        }
    }

    void UART::write_pose(Position* pos, Quaternion* quat)
    {
        std::string data = "";
        data += (std::to_string(pos->x) + ',');
        data += (std::to_string(pos->y) + ',');
        data += (std::to_string(pos->z) + ',');
        data += (std::to_string(quat->w) + ',');
        data += (std::to_string(quat->x) + ',');
        data += (std::to_string(quat->y) + ',');
        data += (std::to_string(quat->z));

        std::cout << "Send : " << data << '\n';

        const char* data_char = data.c_str();
        int len = data.length();

        ssize_t bytes_written = write(uart_fd, data_char, len);

        if (bytes_written == -1) {
            perror("Error writing to UART");
            close(uart_fd);
            exit(EXIT_FAILURE);
        }
    }


    UART::~UART()
    {
        close(uart_fd);
    }
}
