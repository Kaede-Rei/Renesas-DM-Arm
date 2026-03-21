#ifndef _s_comms_h_
#define _s_comms_h_

#include <stdint.h>
#include <stddef.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef enum {
    COMMS_SUCCESS = 0,
    COMMS_ERROR_INVALID_PACKET,
} CommsErrorCode;

typedef struct {
    uint8_t id;
    uint16_t x;
    uint16_t y;
    float confidence;
} WeedData;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CommsErrorCode s_comms_process_packet(const uint8_t* packet, size_t length);
CommsErrorCode s_comms_get_weed_data(WeedData* data);

#endif
