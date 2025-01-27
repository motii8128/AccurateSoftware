#include "serial_controller/serial_handler.hpp"

namespace serial_controller
{
    SerialHandler::SerialHandler()
    {

    }

    bool SerialHandler::OpenPort(std::string port_path)
    {
        port_path_ = port_path;
        fd_ =open(port_path.c_str(), O_RDWR | O_NOCTTY);
        struct termios conf_tio;
        tcgetattr(fd_,&conf_tio);
        speed_t BAUDRATE = B115200;
        cfsetispeed(&conf_tio, BAUDRATE);
        cfsetospeed(&conf_tio, BAUDRATE);
        conf_tio.c_lflag &= ~(ECHO | ICANON);
        conf_tio.c_cc[VMIN]=1;
        conf_tio.c_cc[VTIME]=1;
        conf_tio.c_cflag &= ~PARENB;
        conf_tio.c_cflag &= ~CSTOPB;
        conf_tio.c_cflag |= CS8;
        tcsetattr(fd_,TCSANOW,&conf_tio);

        if(fd_ < 0)
        {
            return false;
        }

        return true;
    }

    void SerialHandler::ClosePort()
    {
        close(fd_);
    }

    bool SerialHandler::WritePort(std::string tx)
    {
        if(fd_ < 0)
        {
            return false;
        }

        int err = write(fd_, tx.c_str(), tx.size());

        if(err > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    std::string SerialHandler::ReadPort()
    {
        if(fd_ < 0)
        {
            return std::string("Read ERROR");
        }
        char buf[100];

        ssize_t bytes_read = read(fd_, buf, sizeof(buf));

        if(bytes_read < 0)
        {
            return std::string("Read ERROR");
        }
        else
        {
            buf[bytes_read] = '\0';
            
            return std::string(buf);
        }
    }
}