#include <sys/stat.h>
#include "hal_data.h"
#include "drive/d_uart.h"

int _write(int file, char* ptr, int len);
int _read(int file, char* ptr, int len);
int _close(int file);
int _fstat(int file, struct stat* st);
int _isatty(int file);
int _lseek(int file, int ptr, int dir);
void _exit(int status);

int _write(int file, char* ptr, int len) {
    if(file != 1 && file != 2) return -1;

    if(len <= 0) return 0;

    fsp_err_t err = g_uart7.p_api->write(g_uart7.p_ctrl, (const uint8_t*)ptr, (uint32_t)len);
    if(err != FSP_SUCCESS) return -1;

    d_uart_wait_tx_complete(UART7);

    return len;
}

int _read(int file, char* ptr, int len) {
    if(file != 0)return -1;

    if(len <= 0) return 0;

    fsp_err_t err = g_uart7.p_api->read(g_uart7.p_ctrl, (uint8_t*)ptr, (uint32_t)len);
    if(err != FSP_SUCCESS) return -1;

    d_uart_wait_rx_complete(UART7);

    _write(1, ptr, len);

    return len;
}

__attribute__((weak)) int _close(int file) { (void)file; return -1; }
__attribute__((weak)) int _fstat(int file, struct stat* st) { (void)file; st->st_mode = S_IFCHR; return 0; }
__attribute__((weak)) int _isatty(int file) { (void)file; return 1; }
__attribute__((weak)) int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return -1; }
__attribute__((weak)) void _exit(int status) { (void)status; while(1); }