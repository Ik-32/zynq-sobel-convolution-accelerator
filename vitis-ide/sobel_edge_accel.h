#ifndef SOBEL_EDGE_ACCEL_H
#define SOBEL_EDGE_ACCEL_H

#include <stdint.h>
#include <stdbool.h>
#include "xil_types.h"
#include "xparameters.h"

#if defined(XPAR_SOBEL_EDGE_PE8X8_0_S_AXI_CONTROL_BASEADDR)
#define SOBEL_EDGE_ACCEL_BASEADDR XPAR_SOBEL_EDGE_PE8X8_0_S_AXI_CONTROL_BASEADDR
#elif defined(XPAR_XSOBEL_EDGE_PE8X8_0_S_AXI_CONTROL_BASEADDR)
#define SOBEL_EDGE_ACCEL_BASEADDR XPAR_XSOBEL_EDGE_PE8X8_0_S_AXI_CONTROL_BASEADDR
#elif defined(XPAR_SOBEL_EDGE_PE8X8_0_CONTROL_BASEADDR)
#define SOBEL_EDGE_ACCEL_BASEADDR XPAR_SOBEL_EDGE_PE8X8_0_CONTROL_BASEADDR
#else
#define SOBEL_EDGE_ACCEL_BASEADDR 0x43C10000U
#endif

/*
 * Register map for HLS top:
 * sobel_edge_pe8x8(input, gx_output, gy_output, input_height, input_width)
 */
#define SOBEL_EDGE_CTRL_OFFSET          0x00U
#define SOBEL_EDGE_GIE_OFFSET           0x04U
#define SOBEL_EDGE_IER_OFFSET           0x08U
#define SOBEL_EDGE_ISR_OFFSET           0x0CU

#define SOBEL_EDGE_INPUT_OFFSET         0x10U
#define SOBEL_EDGE_INPUT_HIGH_OFFSET    0x14U
#define SOBEL_EDGE_GX_OFFSET            0x1CU
#define SOBEL_EDGE_GX_HIGH_OFFSET       0x20U
#define SOBEL_EDGE_GY_OFFSET            0x28U
#define SOBEL_EDGE_GY_HIGH_OFFSET       0x2CU
#define SOBEL_EDGE_HEIGHT_OFFSET        0x34U
#define SOBEL_EDGE_WIDTH_OFFSET         0x3CU

#define SOBEL_EDGE_AP_START_MASK        0x01U
#define SOBEL_EDGE_AP_DONE_MASK         0x02U
#define SOBEL_EDGE_AP_IDLE_MASK         0x04U
#define SOBEL_EDGE_AP_READY_MASK        0x08U
#define SOBEL_EDGE_GIE_ENABLE_MASK      0x01U
#define SOBEL_EDGE_IER_INT_EN_MASK      0x01U
#define SOBEL_EDGE_ISR_INT_CLR_MASK     0x01U

void sobel_edge_accel_init(UINTPTR baseaddr);
void sobel_edge_accel_start(UINTPTR input_ptr,
                            UINTPTR gx_output_ptr,
                            UINTPTR gy_output_ptr,
                            int input_height,
                            int input_width);
bool sobel_edge_accel_is_done(void);
void sobel_edge_accel_wait_done(void);
void sobel_edge_accel_interrupt_enable(void);
void sobel_edge_accel_interrupt_clear(void);
UINTPTR sobel_edge_accel_get_baseaddr(void);

#endif
