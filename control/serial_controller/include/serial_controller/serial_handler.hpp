#ifndef SERIAL_HANDLER_HPP_
#define SERIAL_HANDLER_HPP_

#include <vector>
#include <string>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>

namespace serial_controller
{
    class SerialHandler
    {
        public:
        SerialHandler();

        bool OpenPort(std::string port_path);
        void ClosePort();
        bool WritePort(std::string tx);
        std::string ReadPort();

        private:
        int fd_;
        std::string port_path_;
    };
}

#endif