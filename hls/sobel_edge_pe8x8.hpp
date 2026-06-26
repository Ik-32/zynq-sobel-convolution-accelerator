#ifndef SOBEL_EDGE_PE8X8_HPP
#define SOBEL_EDGE_PE8X8_HPP

#include "conv3x3_pe8x8.hpp"

#ifndef CONV_MAX_INPUT_ELEMS
#define CONV_MAX_INPUT_ELEMS (CONV_MAX_INPUT_HEIGHT * CONV_MAX_INPUT_WIDTH)
#endif

#ifndef CONV_MAX_OUTPUT_ELEMS
#define CONV_MAX_OUTPUT_ELEMS (CONV_MAX_INPUT_HEIGHT * CONV_MAX_INPUT_WIDTH)
#endif

#ifndef SOBEL_MAX_INPUT_ELEMS
#define SOBEL_MAX_INPUT_ELEMS CONV_MAX_INPUT_ELEMS
#endif

#ifndef SOBEL_MAX_OUTPUT_ELEMS
#define SOBEL_MAX_OUTPUT_ELEMS CONV_MAX_OUTPUT_ELEMS
#endif

extern "C" void sobel_edge_pe8x8(
    const data_t_u32 *input,
    sobel_conv_t *gx_output,
    sobel_conv_t *gy_output,
    int input_height,
    int input_width
);

#endif
