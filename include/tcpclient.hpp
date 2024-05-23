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

class TCPClient
{
public:
    TCPClient();
    ~TCPClient();

    bool connect(const std::string &hostname, int port = 8888);
    bool close();
    std::string get_hostname() const;
    int get_port() const;
    void send_pose(const EMIRO::Pose &pose);
    void read_response();

private:
    boost::asio::io_service _io_service;
    boost::asio::ip::tcp::socket _socket;
    boost::asio::ip::tcp::resolver _resolver;
    std::string hostname;
    int port;
};
