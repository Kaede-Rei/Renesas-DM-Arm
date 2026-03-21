/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_NUM_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_uart_rxi_isr, /* SCI7 RXI (Receive data full) */
            [1] = sci_uart_txi_isr, /* SCI7 TXI (Transmit data empty) */
            [2] = sci_uart_tei_isr, /* SCI7 TEI (Transmit end) */
            [3] = sci_uart_eri_isr, /* SCI7 ERI (Receive error) */
            [4] = canfd_error_isr, /* CAN0 CHERR (Channel  error) */
            [5] = canfd_channel_tx_isr, /* CAN0 TX (Transmit interrupt) */
            [6] = canfd_common_fifo_rx_isr, /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [7] = canfd_error_isr, /* CAN GLERR (Global error) */
            [8] = canfd_rx_fifo_isr, /* CAN RXF (Global receive FIFO interrupt) */
            [9] = sci_uart_rxi_isr, /* SCI6 RXI (Receive data full) */
            [10] = sci_uart_txi_isr, /* SCI6 TXI (Transmit data empty) */
            [11] = sci_uart_tei_isr, /* SCI6 TEI (Transmit end) */
            [12] = sci_uart_eri_isr, /* SCI6 ERI (Receive error) */
        };
        #if BSP_FEATURE_ICU_HAS_IELSR
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_NUM_ENTRIES] =
        {
            [0] = BSP_PRV_VECT_ENUM(EVENT_SCI7_RXI,GROUP0), /* SCI7 RXI (Receive data full) */
            [1] = BSP_PRV_VECT_ENUM(EVENT_SCI7_TXI,GROUP1), /* SCI7 TXI (Transmit data empty) */
            [2] = BSP_PRV_VECT_ENUM(EVENT_SCI7_TEI,GROUP2), /* SCI7 TEI (Transmit end) */
            [3] = BSP_PRV_VECT_ENUM(EVENT_SCI7_ERI,GROUP3), /* SCI7 ERI (Receive error) */
            [4] = BSP_PRV_VECT_ENUM(EVENT_CAN0_CHERR,GROUP4), /* CAN0 CHERR (Channel  error) */
            [5] = BSP_PRV_VECT_ENUM(EVENT_CAN0_TX,GROUP5), /* CAN0 TX (Transmit interrupt) */
            [6] = BSP_PRV_VECT_ENUM(EVENT_CAN0_COMFRX,GROUP6), /* CAN0 COMFRX (Common FIFO receive interrupt) */
            [7] = BSP_PRV_VECT_ENUM(EVENT_CAN_GLERR,GROUP7), /* CAN GLERR (Global error) */
            [8] = BSP_PRV_VECT_ENUM(EVENT_CAN_RXF,GROUP0), /* CAN RXF (Global receive FIFO interrupt) */
            [9] = BSP_PRV_VECT_ENUM(EVENT_SCI6_RXI,GROUP1), /* SCI6 RXI (Receive data full) */
            [10] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TXI,GROUP2), /* SCI6 TXI (Transmit data empty) */
            [11] = BSP_PRV_VECT_ENUM(EVENT_SCI6_TEI,GROUP3), /* SCI6 TEI (Transmit end) */
            [12] = BSP_PRV_VECT_ENUM(EVENT_SCI6_ERI,GROUP4), /* SCI6 ERI (Receive error) */
        };
        #endif
        #endif