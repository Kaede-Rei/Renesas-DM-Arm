#include "d_can.h"

// ! ========================= 变 量 声 明 ========================= ! //

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



// ! ========================= 接 口 函 数 实 现 ========================= ! //



// ! ========================= 私 有 函 数 实 现 ========================= ! //

void canfd0_callback(can_callback_args_t* p_args) {
    switch(p_args->event) {
        // 当发送完成时，CAN事件将被触发
        case CAN_EVENT_TX_COMPLETE:
        {

            break;
        }
        // 当接收完成时，CAN事件将被触发
        case CAN_EVENT_RX_COMPLETE:
        {

            break;
        }
        default:break;
    }
}
