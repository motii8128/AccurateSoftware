#ifndef UDP_UTILS_HPP_
#define UDP_UTILS_HPP_

#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

namespace udp_connector_ros2
{
    class UdpHandler
    {
        public:
        UdpHandler();

        void recv();
        void send();

        sockaddr_in getDestAddr();
        std::string getRecvData();

        private:
        sockaddr_in server_addr_, client_addr_;
        std::string recv_data_;
    };
}

#endif