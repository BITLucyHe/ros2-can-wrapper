#pragma once

#include <stdint.h>

// example send packet
struct msg_send {
    uint8_t any_state = 0;

} __attribute__((packed));

// example receive packet
struct msg_receive {
    uint8_t any_data = 0;

} __attribute__((packed));
