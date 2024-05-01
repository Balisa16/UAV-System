#pragma once

#include <types.hpp>
#include <fcntl.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <termios.h>
#include <unistd.h>

namespace EMIRO
{
  class UART
  {
  public:
    UART();

    void init(std::string device = "/dev/ttyTHS1", speed_t baudrate = B9600);

    // Receive
    /**
     * @brief Read data from UART in string format
     *
     * @param length length of data to read
     * @return std::string
     *
     * @default length = 1024
     */
    std::string read_s(size_t length = 1024);

    /**
     * @brief Read data from UART in pose format (Posirion, Quaternion)
     *
     * @param pos Position which will be filled
     * @param quat Quaternion which will be filled
     *
     * @return true Success read without error
     * @return false  Error while reading
     */
    bool read_pose(Position *pos, Quaternion *quat);

    // Send
    /**
     * @brief Write data into UART in string format
     *
     * @param data word to send
     */
    void write_s(std::string data = "None");

    /**
     * @brief write data into UART in pose format (Posirion, Quaternion)
     *
     * @param pos Position to send
     * @param quat Quaternion to send
     */
    void write_pose(Position *pos, Quaternion *quat);

    ~UART();

  private:
    int uart_fd;
    std::string device;
    speed_t baudrate;
  };
} // namespace EMIRO