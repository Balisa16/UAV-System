#pragma once

#include <iostream>
#include <boost/asio.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <chrono>
#include <random>
#include <types.hpp>

using boost::asio::ip::tcp;

class TCPClient
{
public:
    TCPClient();
    ~TCPClient();
    bool connect(const std::string &hostname = "localhost", int port = 8888);
    bool close();
    std::string get_hostname() const;
    int get_port() const;
    void send_pose(const EMIRO::Pose &pose);
    void read_response();

private:
    std::string hostname;
    int port;
    boost::asio::io_context _io_context;
    tcp::socket _socket;
    tcp::resolver _resolver;
};