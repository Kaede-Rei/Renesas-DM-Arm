#ifndef _d_can_h_
#define _d_can_h_

#include "hal_data.h"
#include "r_can_api.h"

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

extern const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM];

typedef enum {
    CAN_SUCCESS = 0,
    CAN_ERROR,
    CAN_NOT_COMPLETE,
    CAN_BUSY,
    CAN_OVERFLOW,
    CAN_EMPTY,
} CanErrorCode_e;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

CanErrorCode_e d_can_init(void);
CanErrorCode_e d_can_tx_complete(void);
CanErrorCode_e d_can_rx_complete(void);
CanErrorCode_e d_can_is_busy(void);
CanErrorCode_e d_can_write(can_instance_t* const can_instance, uint32_t id, uint8_t* data, uint8_t length);
CanErrorCode_e d_can_read(can_instance_t* const can_instance, can_frame_t* const frame);

#endif
