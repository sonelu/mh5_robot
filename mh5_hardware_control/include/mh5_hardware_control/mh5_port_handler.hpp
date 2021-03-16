#ifndef MH5_DYNAMIXEL_PORTHANDLER_H_
#define MH5_DYNAMIXEL_PORTHANDLER_H_

#include <dynamixel_sdk/port_handler.h>

namespace mh5_port_handler
{

#if defined(__linux__)
#include <dynamixel_sdk/port_handler_linux.h>
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


class PortHandlerMH5: public PARENT
{
#if defined (__linux__)
    bool setRS485() {
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
    }

#else
    bool setRS485() {
        ROS_WARN("'setRS485' only implemented for Linux; will be ignored");
        return true;
    }

#endif

}

}
#endif /* MH5_DYNAMIXEL_PORTHANDLER_H_ */