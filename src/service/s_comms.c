#include "s_comms.h"

#include <string.h>
#include <stdlib.h>


// ! ========================= 变 量 声 明 ========================= ! //

WeedData weed_comms = { 0 };

// 单例结构体实现
const struct CommsInterface s_comms_instance = {
    .process_packet = s_comms_process_packet,
    .get_weed_data = s_comms_get_weed_data
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static CommsErrorCode process_weed_packet(const uint8_t* packet, size_t length);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

CommsErrorCode s_comms_process_packet(const uint8_t* packet, size_t length) {
    if(memcmp((const char*)packet, "WEED:", 5) == 0) return process_weed_packet(packet, length);

    return COMMS_ERROR_INVALID_PACKET;
}

CommsErrorCode s_comms_get_weed_data(WeedData* data) {
    if(data == NULL) return COMMS_ERROR_INVALID_PACKET;

    *data = weed_comms;
    return COMMS_SUCCESS;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

static CommsErrorCode process_weed_packet(const uint8_t* packet, size_t length) {
    if(packet == NULL || length == 0) return COMMS_ERROR_INVALID_PACKET;

    uint8_t temp_buf[length + 1];
    uint8_t* end = temp_buf + length;
    memcpy(temp_buf, packet, length);
    temp_buf[length] = '\0';

    uint8_t* id = memchr(temp_buf, ':', length);
    if(!id || id >= end + 1) return COMMS_ERROR_INVALID_PACKET;
    id++;

    uint8_t* x = memchr(id, ',', (unsigned int)(end - id));
    if(!x || x >= end + 1) return COMMS_ERROR_INVALID_PACKET;
    x++;

    uint8_t* y = memchr(x, ',', (unsigned int)(end - x));
    if(!y || y >= end + 1) return COMMS_ERROR_INVALID_PACKET;
    y++;

    uint8_t* confidence = memchr(y, ',', (unsigned int)(end - y));
    if(!confidence || confidence >= end + 1) return COMMS_ERROR_INVALID_PACKET;
    confidence++;

    weed_comms.id = (uint8_t)atoi((const char*)id);
    weed_comms.x = (uint16_t)atoi((const char*)x);
    weed_comms.y = (uint16_t)atoi((const char*)y);
    weed_comms.confidence = (float)atof((const char*)confidence);

    return COMMS_SUCCESS;
}
