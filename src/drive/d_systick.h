#ifndef _d_systick_h_
#define _d_systick_h_

#include "hal_data.h"   // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

typedef uint32_t ms_t;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

fsp_err_t d_systick_init(void);
ms_t d_systick_get_ms(void);
ms_t d_systick_get_s(void);
bool d_systick_is_timeout(ms_t start, ms_t timeout_ms);

#endif
