#include "sobel_edge_accel.h"
#include "xil_io.h"

static UINTPTR base_addr = SOBEL_EDGE_ACCEL_BASEADDR;

static void sobel_edge_write_addr(uint32_t low_offset, uint32_t high_offset, UINTPTR addr)
{
    Xil_Out32(base_addr + low_offset, (uint32_t)((uint64_t)addr & 0xFFFFFFFFu));
    Xil_Out32(base_addr + high_offset, (uint32_t)(((uint64_t)addr >> 32) & 0xFFFFFFFFu));
}

void sobel_edge_accel_init(UINTPTR baseaddr)
{
    base_addr = baseaddr;
}

UINTPTR sobel_edge_accel_get_baseaddr(void)
{
    return base_addr;
}

void sobel_edge_accel_start(UINTPTR input_ptr,
                            UINTPTR gx_output_ptr,
                            UINTPTR gy_output_ptr,
                            int input_height,
                            int input_width)
{
    sobel_edge_write_addr(SOBEL_EDGE_INPUT_OFFSET,
                          SOBEL_EDGE_INPUT_HIGH_OFFSET,
                          input_ptr);
    sobel_edge_write_addr(SOBEL_EDGE_GX_OFFSET,
                          SOBEL_EDGE_GX_HIGH_OFFSET,
                          gx_output_ptr);
    sobel_edge_write_addr(SOBEL_EDGE_GY_OFFSET,
                          SOBEL_EDGE_GY_HIGH_OFFSET,
                          gy_output_ptr);

    Xil_Out32(base_addr + SOBEL_EDGE_HEIGHT_OFFSET, (uint32_t)input_height);
    Xil_Out32(base_addr + SOBEL_EDGE_WIDTH_OFFSET,  (uint32_t)input_width);

    Xil_Out32(base_addr + SOBEL_EDGE_CTRL_OFFSET, SOBEL_EDGE_AP_START_MASK);
}

bool sobel_edge_accel_is_done(void)
{
    return (Xil_In32(base_addr + SOBEL_EDGE_CTRL_OFFSET) & SOBEL_EDGE_AP_DONE_MASK) != 0u;
}

void sobel_edge_accel_wait_done(void)
{
    while(!sobel_edge_accel_is_done()) {
        /* busy wait */
    }
}

void sobel_edge_accel_interrupt_enable(void)
{
    Xil_Out32(base_addr + SOBEL_EDGE_GIE_OFFSET, SOBEL_EDGE_GIE_ENABLE_MASK);
    Xil_Out32(base_addr + SOBEL_EDGE_IER_OFFSET, SOBEL_EDGE_IER_INT_EN_MASK);
}

void sobel_edge_accel_interrupt_clear(void)
{
    Xil_Out32(base_addr + SOBEL_EDGE_ISR_OFFSET, SOBEL_EDGE_ISR_INT_CLR_MASK);
}
