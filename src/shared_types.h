
#ifndef SHARED_TYPES_H_
#define SHARED_TYPES_H_

#include <stdint.h>

// Packets structure for emergency transmission over CAN bus
typedef struct {
    uint8_t node_id;      
    uint8_t severity;     // 1 = Info, 2 = Warning, 3 = Critical Emergency
    uint16_t error_code;  
} __attribute__((packed)) diagnostic_packet_t;

#endif