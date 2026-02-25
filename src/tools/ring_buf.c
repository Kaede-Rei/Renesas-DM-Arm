#include "ring_buf.h"

#include <stddef.h>

// ! ========================= 变 量 声 明 ========================= ! //



// ! ========================= 私 有 函 数 声 明 ========================= ! //

static RingBufError_te _write(RingBuf_t* const self, const uint8_t data);
static RingBufError_te _read(RingBuf_t* const self, uint8_t* const data);
static RingBufError_te _clear(RingBuf_t* const self);

static int8_t _is_full(RingBuf_t* const self);
static int8_t _is_empty(RingBuf_t* const self);

static int16_t _get_size(RingBuf_t* const self);
static int16_t _get_capacity(RingBuf_t* const self);

// ! ========================= 接 口 函 数 实 现 ========================= ! //

/**
 * @brief 环形缓冲区构造函数
 * @param self 指向 RingBuf_t 结构体的指针
 * @param buf 指向用于存储数据的缓冲区的指针
 * @param capacity 缓冲区的容量
 * @param overwrite 是否允许覆盖旧数据，1 表示允许，0 表示不允许
 * @return RingBufError_te 枚举类型，表示操作结果
 */
RingBufError_te RingBuf(RingBuf_t* const self, uint8_t* const buf, const uint16_t capacity, const uint8_t overwrite) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(buf == NULL) return RING_BUF_ERR_NULL_PTR;
    if(capacity == 0) return RING_BUF_ERR_FULL;

    self->write = _write;
    self->read = _read;
    self->clear = _clear;
    self->is_full = _is_full;
    self->is_empty = _is_empty;
    self->get_size = _get_size;
    self->get_capacity = _get_capacity;

    self->_buf_ = buf;
    self->_size_ = 0;
    self->_capacity_ = capacity;

    self->_write_idx_ = 0;
    self->_read_idx_ = 0;

    self->_overwrite_ = overwrite;
    self->_using_ = 0;

    return RING_BUF_OK;
}

// ! ========================= 私 有 函 数 实 现 ========================= ! //

/**
 * @brief 向环形缓冲区写入数据
 * @param self 指向 RingBuf_t 结构体的指针
 * @param data 要写入的数据
 * @return RingBufError_te 枚举类型，表示操作结果
 */
RingBufError_te _write(RingBuf_t* const self, const uint8_t data) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(self->_using_) return RING_BUF_ERR_IN_USE;

    self->_using_ = 1;
    if(self->_overwrite_) {
        self->_buf_[self->_write_idx_] = data;
        self->_write_idx_ = (uint16_t)((self->_write_idx_ + 1) % self->_capacity_);
        if(self->is_full(self)) self->_read_idx_ = (uint16_t)((self->_read_idx_ + 1) % self->_capacity_);
        else self->_size_++;
    }
    else {
        if(self->is_full(self)) return RING_BUF_ERR_FULL;
        self->_buf_[self->_write_idx_] = data;
        self->_write_idx_ = (uint16_t)((self->_write_idx_ + 1) % self->_capacity_);
        self->_size_++;
    }
    self->_using_ = 0;

    return RING_BUF_OK;
}

/**
 * @brief 从环形缓冲区读取数据
 * @param self 指向 RingBuf_t 结构体的指针
 * @param data 指向存储读取数据的变量的指针
 * @return RingBufError_te 枚举类型，表示操作结果
 */
RingBufError_te _read(RingBuf_t* const self, uint8_t* const data) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(data == NULL) return RING_BUF_ERR_NULL_PTR;
    if(self->_using_) return RING_BUF_ERR_IN_USE;

    self->_using_ = 1;
    if(self->is_empty(self)) {
        self->_using_ = 0;
        return RING_BUF_ERR_EMPTY;
    }
    *data = self->_buf_[self->_read_idx_];
    self->_read_idx_ = (uint16_t)((self->_read_idx_ + 1) % self->_capacity_);
    self->_size_--;
    self->_using_ = 0;

    return RING_BUF_OK;
}

/**
 * @brief 清空环形缓冲区
 * @param self 指向 RingBuf_t 结构体的指针
 * @return RingBufError_te 枚举类型，表示操作结果
 */
RingBufError_te _clear(RingBuf_t* const self) {
    if(self == NULL) return RING_BUF_ERR_NULL_PTR;
    if(self->_using_) return RING_BUF_ERR_IN_USE;

    self->_using_ = 1;
    self->_size_ = 0;
    self->_write_idx_ = 0;
    self->_read_idx_ = 0;
    self->_using_ = 0;

    return RING_BUF_OK;
}

/**
 * @brief 检查环形缓冲区是否已满
 * @param self 指向 RingBuf_t 结构体的指针
 * @return 1 表示已满，0 表示未满，-1 表示错误
 */
int8_t _is_full(RingBuf_t* const self) {
    if(self == NULL) return -1;

    return (self->_size_ >= self->_capacity_) ? 1 : 0;
}

/**
 * @brief 检查环形缓冲区是否为空
 * @param self 指向 RingBuf_t 结构体的指针
 * @return 1 表示为空，0 表示非空，-1 表示错误
 */
int8_t _is_empty(RingBuf_t* const self) {
    if(self == NULL) return -1;

    return (self->_size_ == 0) ? 1 : 0;
}

/**
 * @brief 获取环形缓冲区当前存储的数据量
 * @param self 指向 RingBuf_t 结构体的指针
 * @return 当前存储的数据量，-1 表示错误
 */
int16_t _get_size(RingBuf_t* const self) {
    if(self == NULL) return -1;

    return (int16_t)(self->_size_);
}

/**
 * @brief 获取环形缓冲区的总容量
 * @param self 指向 RingBuf_t 结构体的指针
 * @return 环形缓冲区的总容量，-1 表示错误
 */
int16_t _get_capacity(RingBuf_t* const self) {
    if(self == NULL) return -1;

    return (int16_t)(self->_capacity_);
}
