#ifndef _ring_buf_h_
#define _ring_buf_h_

#include <stdint.h>

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 环形缓冲区错误枚举类型
 * @note RING_BUF_OK: 操作成功
 * @note RING_BUF_ERR_NULL_PTR: 指针为 NULL 错误
 * @note RING_BUF_ERR_IN_USE: 缓冲区正被使用错误
 * @note RING_BUF_ERR_FULL: 缓冲区已满错误
 * @note RING_BUF_ERR_EMPTY: 缓冲区为空错误
 */
typedef enum {
    RING_BUF_OK = 0,
    RING_BUF_ERR_NULL_PTR = -1,
    RING_BUF_ERR_IN_USE = -2,
    RING_BUF_ERR_FULL = -3,
    RING_BUF_ERR_EMPTY = -4,
} RingBufError_te;

typedef struct RingBuf_t RingBuf_t;
struct RingBuf_t {
/// public:
    /**
     * @brief 向环形缓冲区写入数据
     * @param self 指向 RingBuf_t 结构体的指针
     * @param data 要写入的数据
     * @return RingBufError_te 枚举类型，表示操作结果
     */
    RingBufError_te(*write)(RingBuf_t* const self, const uint8_t data);
    /**
     * @brief 从环形缓冲区读取数据
     * @param self 指向 RingBuf_t 结构体的指针
     * @param data 指向存储读取数据的变量的指针
     * @return RingBufError_te 枚举类型，表示操作结果
     */
    RingBufError_te(*read)(RingBuf_t* const self, uint8_t* const data);
    /**
     * @brief 清空环形缓冲区
     * @param self 指向 RingBuf_t 结构体的指针
     * @return RingBufError_te 枚举类型，表示操作结果
     */
    RingBufError_te(*clear)(RingBuf_t* const self);

    /**
     * @brief 检查环形缓冲区是否已满
     * @param self 指向 RingBuf_t 结构体的指针
     * @return 1 表示已满，0 表示未满，-1 表示错误
     */
    int8_t(*is_full)(RingBuf_t* const self);
    /**
     * @brief 检查环形缓冲区是否为空
     * @param self 指向 RingBuf_t 结构体的指针
     * @return 1 表示为空，0 表示非空，-1 表示错误
     */
    int8_t(*is_empty)(RingBuf_t* const self);

    /**
     * @brief 获取环形缓冲区当前存储的数据量
     * @param self 指向 RingBuf_t 结构体的指针
     * @return 当前存储的数据量，-1 表示错误
     */
    int16_t(*get_size)(RingBuf_t* const self);
    /**
     * @brief 获取环形缓冲区的总容量
     * @param self 指向 RingBuf_t 结构体的指针
     * @return 环形缓冲区的总容量，-1 表示错误
     */
    int16_t(*get_capacity)(RingBuf_t* const self);

/// private:
    // 缓冲区指针
    uint8_t* _buf_;
    // 缓冲区当前数据量
    uint16_t _size_;
    // 缓冲区总容量
    uint16_t _capacity_;

    // 写入索引
    uint16_t _write_idx_;
    // 读取索引
    uint16_t _read_idx_;

    // 是否允许覆盖旧数据
    uint8_t _overwrite_;
    // 是否正在使用中
    uint8_t _using_;
};

// ! ========================= 接 口 函 数 声 明 ========================= ! //

RingBufError_te RingBuf(RingBuf_t* const self, uint8_t* const buf, const uint16_t capacity, const uint8_t overwrite);

#endif
