#ifndef CONV3X3_PE8X8_HPP
#define CONV3X3_PE8X8_HPP

#include <stdint.h>

#define CONV_MODE_GRAY 1
#define CONV_MODE_COLOR 4

#ifndef CONV_IMAGE_MODE
#define CONV_IMAGE_MODE CONV_MODE_GRAY
#endif

#if (CONV_IMAGE_MODE != CONV_MODE_GRAY) && (CONV_IMAGE_MODE != CONV_MODE_COLOR)
#error "CONV_IMAGE_MODE must be CONV_MODE_GRAY or CONV_MODE_COLOR"
#endif

#define CONV_ACTIVE_CHANNELS CONV_IMAGE_MODE

#define CONV_MIN_INPUT_HEIGHT 1
#define CONV_MIN_INPUT_WIDTH 1
#define CONV_MAX_INPUT_HEIGHT 2048
#define CONV_MAX_INPUT_WIDTH 2048

#define CONV_MAX_INPUT_SIZE CONV_MAX_INPUT_WIDTH
#define CONV_MIN_INPUT_SIZE 1

#define CONV_KERNEL_SIZE 3
#define CONV_KERNEL_ELEMS (CONV_KERNEL_SIZE * CONV_KERNEL_SIZE)
#define CONV_TOTAL_WEIGHT_ELEMS (CONV_ACTIVE_CHANNELS * CONV_KERNEL_ELEMS)
#define CONV_SAME_PADDING 1
#define CONV_MAX_PADDING CONV_SAME_PADDING

#define CONV_PE_ROWS 8
#define CONV_PE_COLS 8

#define CONV_INPUT_TILE_ROWS (CONV_PE_ROWS + CONV_KERNEL_SIZE - 1)
#define CONV_INPUT_TILE_COLS (CONV_PE_COLS + CONV_KERNEL_SIZE - 1)
#define CONV_LINEBUF_COLS (CONV_MAX_INPUT_WIDTH + (2 * CONV_SAME_PADDING))
#define CONV_MAX_INPUT_ELEMS (CONV_MAX_INPUT_HEIGHT * CONV_MAX_INPUT_WIDTH)
#define CONV_MAX_OUTPUT_ELEMS (CONV_MAX_INPUT_HEIGHT * CONV_MAX_INPUT_WIDTH)

#define CONV_COLOR_C0_SHIFT 0
#define CONV_COLOR_C1_SHIFT 8
#define CONV_COLOR_C2_SHIFT 16
#define CONV_COLOR_C3_SHIFT 24

typedef uint32_t data_t_u32;
typedef uint32_t channel_t_u32;
typedef int8_t weight_t_i8;
typedef int32_t acc_t_i64;
typedef uint8_t edge_t_u8;
typedef int32_t sobel_conv_t;

struct Conv3x3Pe8x8PE
{
    acc_t_i64 sum_reg;

    void reset();
    void step(channel_t_u32 pixel_in, weight_t_i8 weight_in);
};

extern "C" void conv3x3_pe8x8(
    const data_t_u32 *input,
    const weight_t_i8 *kernel,
    acc_t_i64 *output,
    int input_size,
    int padding
);

#endif
