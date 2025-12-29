#pragma once

#include <optional>

#include <linux/can.h>
#include <net/if.h>

#include "protocol.hpp"


class CanSocket {

public:
    CanSocket(const char* can_interface, int can_filter_id, int can_send_id);
    ~CanSocket() = default;

    void send(msg_send packet);
    std::optional<msg_receive> receive();

private:
    int s;
    int enable_canfd = 1;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_filter rfilter[1]; // CAN ID filters (supports multiple)

    canfd_frame send_frame {
        .can_id = 0,
        .len = sizeof(msg_send),
        .flags = CANFD_BRS, // enable bit rate switch
    };

    canfd_frame receive_frame;

};
