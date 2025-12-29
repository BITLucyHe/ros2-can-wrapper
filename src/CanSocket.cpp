#include "can/CanSocket.hpp"

#include <cstring>
#include <stdexcept>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>


#define RUN_CHECK(call, errmsg)                 \
    do {                                        \
        if ((call) < 0) {                       \
            throw std::runtime_error(errmsg);   \
        }                                       \
    } while(0)


CanSocket::CanSocket(const char* can_interface, int can_filter_id, int can_send_id) {
    RUN_CHECK(
        this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW),
        "Failed to create CAN socket"
    );

    // Set socket to non-blocking mode
    int flags;
    RUN_CHECK(
        flags = fcntl(this->s, F_GETFL, 0),
        "fcntl F_GETFL failed"
    );
    RUN_CHECK(
        fcntl(this->s, F_SETFL, flags | O_NONBLOCK),
        "fcntl F_SETFL O_NONBLOCK failed"
    );

    RUN_CHECK(
        setsockopt(this->s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &this->enable_canfd, sizeof(this->enable_canfd)),
        "Failed to enable CAN FD frames"
    );

    this->rfilter[0].can_id = can_filter_id;
    // Standard frame CAN_SFF_MASK, Extended frame CAN_EFF_MASK
    this->rfilter[0].can_mask = CAN_SFF_MASK;
    RUN_CHECK(
        setsockopt(this->s, SOL_CAN_RAW, CAN_RAW_FILTER, &this->rfilter,sizeof(this->rfilter)),
        "Failed to set CAN filter"
    );

    this->send_frame.can_id = can_send_id;

    strcpy(this->ifr.ifr_name, can_interface);
    RUN_CHECK(
        ioctl(this->s, SIOCGIFINDEX, &this->ifr),
        "ioctl failed"
    );

    this->addr.can_family = AF_CAN;
    this->addr.can_ifindex = this->ifr.ifr_ifindex;
    RUN_CHECK(
        bind(this->s, (struct sockaddr *)&this->addr, sizeof(this->addr)),
        "socket bind failed"
    );
}

void CanSocket::send(msg_send packet) {
    memcpy(send_frame.data, &packet, sizeof(packet));
    int nbytes = write(this->s, &send_frame, sizeof(send_frame));

    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            fprintf(stderr, "[CanSocket] Warning: CAN send buffer is full, packet dropped.\n");
        } else {
            throw std::runtime_error("Failed to send CAN message");
        }
    }
}

std::optional<msg_receive> CanSocket::receive() {
    int nbytes = read(this->s, &receive_frame, sizeof(receive_frame));
    if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return std::nullopt;
        }
        throw std::runtime_error("Failed to receive CAN message");
    }

    msg_receive packet;
    memcpy(&packet, receive_frame.data, sizeof(msg_receive));
    return packet;
}
