#include "d_can.h"
#include "hal_data.h"
#include "r_can_api.h"

#include "service/s_delay.h"

#include <string.h>

// ! ========================= 变 量 声 明 ========================= ! //

// can rx 环形缓冲区容量
#define CAN_RX_RING_CAPACITY    16

/**
 * @brief CAN 接收环形缓冲区结构体
 * @param frames 存储 CAN 帧的数组
 * @param write_idx 写入索引
 * @param read_idx 读取索引
 * @param size 当前存储的 CAN 帧数量
 */
typedef struct {
    can_frame_t frames[CAN_RX_RING_CAPACITY];
    volatile uint16_t write_idx;
    volatile uint16_t read_idx;
    volatile uint16_t size;
} CanRxRing_t;

/**
 * @brief CANFD 控制块结构体
 * @param rx_ring CAN 接收环形缓冲区
 * @param tx_complete 发送完成标志
 * @param rx_complete 接收完成标志
 * @param is_busy CAN 是否忙碌标志
 * @param rx_overflow 接收溢出标志
 */
typedef struct {
    CanRxRing_t rx_ring;
    volatile bool tx_complete;
    volatile bool rx_complete;
    volatile bool is_busy;
    volatile bool rx_overflow;
} CanfdCb_t;
static CanfdCb_t canfd0_cb;

/**
 * @brief 官方要求用户必须定义一个全局的 CANFD 接收过滤器数组
 * @param id.id CAN ID，帧类型和 ID 模式
 * @param id.frame_type CAN 帧类型，数据帧或远程帧
 * @param id.id_mode CAN ID 模式，标准或扩展
 * @param mask.mask_id 在筛选消息时要比较的 ID 位的掩码
 * @param mask.mask_frame_type 在筛选消息时要比较的帧类型位
 * @param mask.mask_id_mode 在筛选消息时要比较的 ID 模式位
 * @param destination.minimum_dlc 如果启用 DLC 检查，任何比以下设置短的消息将被拒绝
 * @param destination.rx_buffer 可选择指定接收消息缓冲区 (RX MB) 来存储已接收的帧 RX MB 不具备中断或覆盖保护，必须使用 R_CANFD_InfoGet 和 R_CANFD_Read 进行检查
 * @param destination.fifo_select_flags 指定要将筛选后的消息发送到哪个 FIFO，可以将多个 FIFO 用 OR 方式组合
 */
const canfd_afl_entry_t p_canfd0_afl[CANFD_CFG_AFL_CH0_RULE_NUM] =
{
    {
        .id =
        {
            // 指定要接受的 ID、ID 类型和帧类型
            .id = 0x000,
            .frame_type = CAN_FRAME_TYPE_DATA,
            .id_mode = CAN_ID_MODE_STANDARD
        },

        .mask =
        {
            // 这些值掩盖在筛选消息时要比较的ID模式位
            .mask_id = 0x000,
            .mask_frame_type = 1,
            .mask_id_mode = 1
        },

        .destination =
        {
            // 如果启用DLC检查，任何比以下设置短的消息将被拒绝
            .minimum_dlc = CANFD_MINIMUM_DLC_0,

            // 可选择指定接收消息缓冲区 (RX MB) 来存储已接收的帧 RX MB 不具备中断或覆盖保护，
            // 必须使用 R CANFD InfoGet 和 R CANFD Read 进行检查
            .rx_buffer = CANFD_RX_MB_NONE,

            // 指定要将筛选后的消息发送到哪个 FIFO，可以将多个 FIFO 用 OR 方式组合
            .fifo_select_flags = CANFD_RX_FIFO_0
        }
    },
};

// ! ========================= 私 有 函 数 声 明 ========================= ! //

static void can_rx_ring_init(CanRxRing_t* const ring);
static CanErrorCode_e can_rx_ring_write(CanRxRing_t* const ring, can_frame_t const* const frame);
static CanErrorCode_e can_rx_ring_read(CanRxRing_t* const ring, can_frame_t* const frame);
static uint16_t can_rx_ring_size(CanRxRing_t const* const ring);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 初始化 CAN 模块
 * @return CanErrorCode_e 枚举类型，表示操作结果
 */
CanErrorCode_e d_can_init(void) {
    // 初始化 CAN 接收环形缓冲区
    can_rx_ring_init(&canfd0_cb.rx_ring);

    // 初始化 canfd0_cb 结构体
    canfd0_cb.tx_complete = false;
    canfd0_cb.rx_complete = false;
    canfd0_cb.is_busy = false;
    canfd0_cb.rx_overflow = false;

    return CAN_SUCCESS;
}

/**
 * @brief 检查 CAN 发送是否完成
 * @return CanErrorCode_e 枚举类型，表示操作结果
 */
CanErrorCode_e d_can_tx_complete(void) {
    uint8_t time_out = 100;
    while(!canfd0_cb.tx_complete && time_out > 0) {
        time_out--;
        s_delay_ms(1);
    }
    CanErrorCode_e result = canfd0_cb.tx_complete ? CAN_SUCCESS : CAN_NOT_COMPLETE;
    canfd0_cb.tx_complete = false;

    return result;
}

/**
 * @brief 检查 CAN 接收是否完成
 * @return CanErrorCode_e 枚举类型，表示操作结果
 */
CanErrorCode_e d_can_rx_complete(void) {
    if(can_rx_ring_size(&canfd0_cb.rx_ring) > 0) {
        return CAN_SUCCESS;
    }

    uint8_t time_out = 100;
    while(!canfd0_cb.rx_complete && time_out > 0) {
        time_out--;
        s_delay_ms(1);
    }
    CanErrorCode_e result = (canfd0_cb.rx_complete || (can_rx_ring_size(&canfd0_cb.rx_ring) > 0)) ? CAN_SUCCESS : CAN_NOT_COMPLETE;
    canfd0_cb.rx_complete = false;

    return result;
}

/**
 * @brief 检查 CAN 是否忙碌
 * @return CanErrorCode_e 枚举类型，表示操作结果
 */
CanErrorCode_e d_can_is_busy(void) {
    return canfd0_cb.is_busy ? CAN_BUSY : CAN_SUCCESS;
}

/**
 * @brief 向 CAN 发送数据帧
 * @param can_instance 指向 CAN 实例的指针
 * @param frame 指向要发送的 CAN 帧的指针
 * @note 此函数使用 CAN 而非 CANFD
 */
CanErrorCode_e d_can_write(can_instance_t* const can_instance, uint32_t id, uint8_t* data, uint8_t length) {
    if(can_instance == NULL || data == NULL) {
        return CAN_ERROR;
    }

    canfd0_cb.is_busy = true;

    can_frame_t frame;
    frame.data_length_code = length;
    frame.type = CAN_FRAME_TYPE_DATA;
    frame.options = 0;
    if(id > 0x7FF) {
        frame.id_mode = CAN_ID_MODE_EXTENDED;
        frame.id = id;
    }
    else {
        frame.id_mode = CAN_ID_MODE_STANDARD;
        frame.id = id & 0x7FF;
    }
    memcpy(frame.data, data, length);

    fsp_err_t err = R_CANFD_Write(can_instance, CANFD_TX_MB_0, &frame);
    if(err != FSP_SUCCESS) {
        canfd0_cb.is_busy = false;
        return CAN_ERROR;
    }

    CanErrorCode_e result = d_can_tx_complete();
    canfd0_cb.is_busy = false;
    return (err == FSP_SUCCESS && result == CAN_SUCCESS) ? CAN_SUCCESS : CAN_ERROR;
}

/**
 * @brief 从 CAN 接收数据帧
 * @param can_instance 指向 CAN 实例的指针
 * @param frame 指向存储接收数据的 CAN 帧结构体的指针
 * @return CanErrorCode_e 枚举类型，表示操作结果
 * @note 此函数使用 CAN 而非 CANFD
 */
CanErrorCode_e d_can_read(can_instance_t* const can_instance, can_frame_t* const frame) {
    if(frame == NULL || can_instance == NULL) {
        return CAN_ERROR;
    }

    if(d_can_rx_complete() != CAN_SUCCESS) {
        return CAN_NOT_COMPLETE;
    }

    CanErrorCode_e result;
    if(can_instance == &g_canfd0) {
        result = can_rx_ring_read(&canfd0_cb.rx_ring, frame);
        if(result == CAN_SUCCESS) canfd0_cb.rx_overflow = false;
    }
    else {
        result = CAN_ERROR;
    }

    return result;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 初始化 CAN 接收环形缓冲区
 * @param ring 指向 CanRxRing_t 结构体的指针
 */
static void can_rx_ring_init(CanRxRing_t* const ring) {
    if(ring == NULL) {
        return;
    }

    ring->write_idx = 0;
    ring->read_idx = 0;
    ring->size = 0;
}

/**
 * @brief 向 CAN 接收环形缓冲区写入数据帧
 * @param ring 指向 CanRxRing_t 结构体的指针
 * @param frame 指向要写入的 CAN 帧的指针
 * @return true 表示写入成功，false 表示写入失败（如缓冲区已满）
 */
static CanErrorCode_e can_rx_ring_write(CanRxRing_t* const ring, can_frame_t const* const frame) {
    uint32_t irq_state;

    if((ring == NULL) || (frame == NULL)) {
        return CAN_ERROR;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    if(ring->size >= CAN_RX_RING_CAPACITY) {
        __set_PRIMASK(irq_state);
        return CAN_OVERFLOW;
    }

    ring->frames[ring->write_idx] = *frame;
    ring->write_idx = (uint16_t)((ring->write_idx + 1U) % CAN_RX_RING_CAPACITY);
    ring->size++;
    __set_PRIMASK(irq_state);

    return true;
}

/**
 * @brief 从 CAN 接收环形缓冲区读取数据帧
 * @param ring 指向 CanRxRing_t 结构体的指针
 * @param frame 指向存储读取数据的 CAN 帧结构体的指针
 * @return true 表示读取成功，false 表示读取失败（如缓冲区为空）
 */
static CanErrorCode_e can_rx_ring_read(CanRxRing_t* const ring, can_frame_t* const frame) {
    uint32_t irq_state;

    if((ring == NULL) || (frame == NULL)) {
        return CAN_ERROR;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();

    if(ring->size == 0) {
        __set_PRIMASK(irq_state);
        return CAN_EMPTY;
    }

    *frame = ring->frames[ring->read_idx];
    ring->read_idx = (uint16_t)((ring->read_idx + 1) % CAN_RX_RING_CAPACITY);
    ring->size--;
    __set_PRIMASK(irq_state);

    return true;
}

/**
 * @brief 获取 CAN 接收环形缓冲区当前存储的数据帧数量
 * @param ring 指向 CanRxRing_t 结构体的指针
 * @return 当前存储的数据帧数量，0 表示缓冲区为空，-1 表示错误
 */
static uint16_t can_rx_ring_size(CanRxRing_t const* const ring) {
    uint16_t size;
    uint32_t irq_state;

    if(ring == NULL) {
        return 0;
    }

    irq_state = __get_PRIMASK();
    __disable_irq();
    size = ring->size;
    __set_PRIMASK(irq_state);

    return size;
}

/**
 * @brief CAN 回调函数
 * @param p_args 指向 CAN 回调参数的指针
 */
void canfd0_callback(can_callback_args_t* p_args) {
    switch(p_args->event) {
        // 当发送完成时，CAN事件将被触发
        case CAN_EVENT_TX_COMPLETE:
        {
            canfd0_cb.tx_complete = true;
            break;
        }
        // 当接收完成时，CAN事件将被触发
        case CAN_EVENT_RX_COMPLETE:
        {
            if(!can_rx_ring_write(&canfd0_cb.rx_ring, &p_args->frame)) {
                canfd0_cb.rx_overflow = true;
            }
            canfd0_cb.rx_complete = true;
            break;
        }
        default:
            break;
    }
}
