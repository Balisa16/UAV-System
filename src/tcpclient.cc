#include <tcpclient.hpp>

TCPClient::TCPClient() : _socket(_io_context),
                         _resolver(_io_context) {}

TCPClient::~TCPClient()
{
    if (_socket.is_open())
        this->close();
}

bool TCPClient::connect(const std::string &hostname, int port)
{
    try
    {
        this->hostname = hostname;
        std::cout << "Connecting to " << hostname << ":" << port << std::endl;
        this->port = port;
        tcp::resolver resolver(_io_context);
        boost::asio::connect(_socket, resolver.resolve(hostname, std::to_string(port)));
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Connection failed: " << e.what() << std::endl;
        return false;
    }
}

bool TCPClient::close()
{
    if (!_socket.is_open())
        return true;
    try
    {
        _socket.close();
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error while closing socket: " << e.what() << std::endl;
        return false;
    }
}

std::string TCPClient::get_hostname() const
{
    return hostname;
}

int TCPClient::get_port() const
{
    return port;
}

void TCPClient::send_pose(const EMIRO::Pose &pose)
{
    try
    {
        std::stringstream ss;
        ss << pose.position.x << "," << pose.position.y << "," << pose.position.z << "," << pose.orientation.w << "," << pose.orientation.x
           << "," << pose.orientation.y << "," << pose.orientation.z << '\n';
        boost::asio::write(_socket, boost::asio::buffer(ss.str()));
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error sending pose: " << e.what() << std::endl;
    }
}

void TCPClient::read_response()
{
    try
    {
        boost::asio::streambuf response;
        boost::asio::read_until(_socket, response, "\n");
        std::istream response_stream(&response);
        std::string response_str;
        std::getline(response_stream, response_str);
        std::cout << "Response: " << response_str << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error reading response: " << e.what() << std::endl;
    }
}
