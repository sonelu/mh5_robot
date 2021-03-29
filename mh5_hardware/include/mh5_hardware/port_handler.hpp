
#include <dynamixel_sdk/port_handler.h>

#pragma once

// forward declaration of PortHandlerMH5 to be able to "friend" it with PortHandlerLinux
namespace mh5_port_handler {
    class PortHandlerMH5;
}

#if defined(__linux__)
#include <linux/serial.h>
#include <sys/ioctl.h>
//hack to access the private members of PortHandlerLinux
#define private friend class mh5_port_handler::PortHandlerMH5; private
#include <dynamixel_sdk/port_handler_linux.h>
#undef private
#define PARENT dynamixel::PortHandlerLinux
#endif

#if defined(__APPLE__)
#include <dynamixel_sdk/port_handler_mac.h>
#define PARENT dynamixel::PortHandlerMac
#endif

#if defined(_WIN32) || defined(_WIN64)
#include <dynamixel_sdk/port_handler_windows.h>
#define PARENT dynamixel::PortHandlerWindows
#endif

namespace mh5_port_handler
{

class PortHandlerMH5: public PARENT
{
    public:

        PortHandlerMH5(const char *port_name) : PARENT(port_name) {}

        bool setRS485() {
    #if defined (__linux__)
            struct serial_rs485 rs485conf;
            /* Enable RS485 mode: */
            rs485conf.flags |= SER_RS485_ENABLED;
            rs485conf.flags |= SER_RS485_RTS_ON_SEND;
            rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);
            rs485conf.delay_rts_before_send = 0;
            rs485conf.delay_rts_after_send = 0;
            if (ioctl (socket_fd_, TIOCSRS485, &rs485conf) < 0)
                return false;
            else
                return true;
    #else
            ROS_WARN("'setRS485' only implemented for Linux; will be ignored");
            return true;
    #endif
        }

};

}
