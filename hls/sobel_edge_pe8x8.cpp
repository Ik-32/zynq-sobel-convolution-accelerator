#include "sobel_edge_pe8x8.hpp"

static bool is_valid_sobel_config(int input_height, int input_width)
{
#pragma HLS INLINE
    if(input_height < CONV_MIN_INPUT_HEIGHT)
        return false;

    if(input_width < CONV_MIN_INPUT_WIDTH)
        return false;

    if(input_height > CONV_MAX_INPUT_HEIGHT)
        return false;

    if(input_width > CONV_MAX_INPUT_WIDTH)
        return false;

    return true;
}

static channel_t_u32 get_gray_channel_from_u32(data_t_u32 pixel)
{
#pragma HLS INLINE
    return (channel_t_u32)(pixel & 0xFFu);
}

static data_t_u32 read_input_same_padding(
    const data_t_u32 *input,
    int input_height,
    int input_width,
    int padded_y,
    int padded_x
)
{
#pragma HLS INLINE
    int in_y = padded_y - CONV_MAX_PADDING;
    int in_x = padded_x - CONV_MAX_PADDING;

    if(in_y < 0 || in_y >= input_height || in_x < 0 || in_x >= input_width)
        return 0;

    return input[in_y * input_width + in_x];
}

static void load_linebuf_rows_8x8(
    const data_t_u32 *input,
    data_t_u32 linebuf[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS],
    int input_height,
    int input_width,
    int out_row_base
)
{
#pragma HLS INLINE off

    int padded_cols = input_width + (2 * CONV_MAX_PADDING);

load_linebuf_y:
    for(int ty = 0; ty < CONV_INPUT_TILE_ROWS; ty++)
    {
    load_linebuf_x:
        for(int px = 0; px < padded_cols; px++)
        {
#pragma HLS PIPELINE II=1
            int padded_y = out_row_base + ty;
            linebuf[ty][px] = read_input_same_padding(input, input_height, input_width, padded_y, px);
        }
    }
}

static void load_ifm_tile_from_linebuf_8x8(
    const data_t_u32 linebuf[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS],
    data_t_u32 ifm_tile[CONV_INPUT_TILE_ROWS][CONV_INPUT_TILE_COLS],
    int input_width,
    int out_col_base
)
{
#pragma HLS INLINE off

    int padded_cols = input_width + (2 * CONV_MAX_PADDING);

load_ifm_y:
    for(int ty = 0; ty < CONV_INPUT_TILE_ROWS; ty++)
    {
    load_ifm_x:
        for(int tx = 0; tx < CONV_INPUT_TILE_COLS; tx++)
        {
#pragma HLS PIPELINE II=1
            int padded_x = out_col_base + tx;

            if(padded_x < padded_cols)
                ifm_tile[ty][tx] = linebuf[ty][padded_x];
            else
                ifm_tile[ty][tx] = 0;
        }
    }
}

static void sobel_conv_xy_from_3x3_window(
    channel_t_u32 p00, channel_t_u32 p01, channel_t_u32 p02,
    channel_t_u32 p10, channel_t_u32 p11, channel_t_u32 p12,
    channel_t_u32 p20, channel_t_u32 p21, channel_t_u32 p22,
    sobel_conv_t *gx_out,
    sobel_conv_t *gy_out
)
{
#pragma HLS INLINE

    (void)p11;

    int32_t i00 = (int32_t)p00;
    int32_t i01 = (int32_t)p01;
    int32_t i02 = (int32_t)p02;
    int32_t i10 = (int32_t)p10;
    int32_t i12 = (int32_t)p12;
    int32_t i20 = (int32_t)p20;
    int32_t i21 = (int32_t)p21;
    int32_t i22 = (int32_t)p22;

    *gx_out = (i02 + (i12 << 1) + i22) - (i00 + (i10 << 1) + i20);
    *gy_out = (i20 + (i21 << 1) + i22) - (i00 + (i01 << 1) + i02);
}

static void compute_sobel_conv_tile_pe8x8(
    const data_t_u32 ifm_tile[CONV_INPUT_TILE_ROWS][CONV_INPUT_TILE_COLS],
    sobel_conv_t gx_tile[CONV_PE_ROWS][CONV_PE_COLS],
    sobel_conv_t gy_tile[CONV_PE_ROWS][CONV_PE_COLS],
    int input_height,
    int input_width,
    int out_row_base,
    int out_col_base
)
{
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable=ifm_tile complete dim=0
#pragma HLS ARRAY_PARTITION variable=gx_tile complete dim=0
#pragma HLS ARRAY_PARTITION variable=gy_tile complete dim=0

compute_y:
    for(int r = 0; r < CONV_PE_ROWS; r++)
    {
#pragma HLS UNROLL

    compute_x:
        for(int c = 0; c < CONV_PE_COLS; c++)
        {
#pragma HLS UNROLL

            bool out_valid = ((out_row_base + r) < input_height) &&
                             ((out_col_base + c) < input_width);

            channel_t_u32 p00 = get_gray_channel_from_u32(ifm_tile[r + 0][c + 0]);
            channel_t_u32 p01 = get_gray_channel_from_u32(ifm_tile[r + 0][c + 1]);
            channel_t_u32 p02 = get_gray_channel_from_u32(ifm_tile[r + 0][c + 2]);

            channel_t_u32 p10 = get_gray_channel_from_u32(ifm_tile[r + 1][c + 0]);
            channel_t_u32 p11 = get_gray_channel_from_u32(ifm_tile[r + 1][c + 1]);
            channel_t_u32 p12 = get_gray_channel_from_u32(ifm_tile[r + 1][c + 2]);

            channel_t_u32 p20 = get_gray_channel_from_u32(ifm_tile[r + 2][c + 0]);
            channel_t_u32 p21 = get_gray_channel_from_u32(ifm_tile[r + 2][c + 1]);
            channel_t_u32 p22 = get_gray_channel_from_u32(ifm_tile[r + 2][c + 2]);

            if(out_valid)
            {
                sobel_conv_xy_from_3x3_window(
                    p00, p01, p02,
                    p10, p11, p12,
                    p20, p21, p22,
                    &gx_tile[r][c],
                    &gy_tile[r][c]
                );
            }
            else
            {
                gx_tile[r][c] = 0;
                gy_tile[r][c] = 0;
            }
        }
    }
}

static void write_back_conv_tile_8x8(
    const sobel_conv_t gx_tile[CONV_PE_ROWS][CONV_PE_COLS],
    const sobel_conv_t gy_tile[CONV_PE_ROWS][CONV_PE_COLS],
    sobel_conv_t *gx_output,
    sobel_conv_t *gy_output,
    int input_height,
    int input_width,
    int out_row_base,
    int out_col_base
)
{
#pragma HLS INLINE off

write_back_y:
    for(int r = 0; r < CONV_PE_ROWS; r++)
    {
    write_back_x:
        for(int c = 0; c < CONV_PE_COLS; c++)
        {
#pragma HLS PIPELINE II=1

            int y = out_row_base + r;
            int x = out_col_base + c;

            if(y < input_height && x < input_width)
            {
                int idx = y * input_width + x;
                gx_output[idx] = gx_tile[r][c];
                gy_output[idx] = gy_tile[r][c];
            }
        }
    }
}

static void compute_write_sobel_conv_rowblock_8x8(
    const data_t_u32 linebuf[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS],
    sobel_conv_t *gx_output,
    sobel_conv_t *gy_output,
    int input_height,
    int input_width,
    int out_row_base
)
{
#pragma HLS INLINE off

    data_t_u32 ifm_tile[CONV_INPUT_TILE_ROWS][CONV_INPUT_TILE_COLS];
    sobel_conv_t gx_tile[CONV_PE_ROWS][CONV_PE_COLS];
    sobel_conv_t gy_tile[CONV_PE_ROWS][CONV_PE_COLS];

#pragma HLS ARRAY_PARTITION variable=ifm_tile complete dim=0
#pragma HLS ARRAY_PARTITION variable=gx_tile complete dim=0
#pragma HLS ARRAY_PARTITION variable=gy_tile complete dim=0

compute_x_tile:
    for(int out_x = 0; out_x < input_width; out_x += CONV_PE_COLS)
    {
        load_ifm_tile_from_linebuf_8x8(
            linebuf,
            ifm_tile,
            input_width,
            out_x
        );

        compute_sobel_conv_tile_pe8x8(
            ifm_tile,
            gx_tile,
            gy_tile,
            input_height,
            input_width,
            out_row_base,
            out_x
        );

        write_back_conv_tile_8x8(
            gx_tile,
            gy_tile,
            gx_output,
            gy_output,
            input_height,
            input_width,
            out_row_base,
            out_x
        );
    }
}

static void prefetch_and_compute_sobel_conv_rowblock_8x8(
    const data_t_u32 *input,
    sobel_conv_t *gx_output,
    sobel_conv_t *gy_output,
    const data_t_u32 current_linebuf[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS],
    data_t_u32 next_linebuf[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS],
    int input_height,
    int input_width,
    int current_out_y,
    int next_out_y,
    bool has_next
)
{
#pragma HLS INLINE off

    if(has_next)
    {
        load_linebuf_rows_8x8(
            input,
            next_linebuf,
            input_height,
            input_width,
            next_out_y
        );
    }

    compute_write_sobel_conv_rowblock_8x8(
        current_linebuf,
        gx_output,
        gy_output,
        input_height,
        input_width,
        current_out_y
    );
}

extern "C" void sobel_edge_pe8x8(
    const data_t_u32 *input,
    sobel_conv_t *gx_output,
    sobel_conv_t *gy_output,
    int input_height,
    int input_width
)
{
#pragma HLS INTERFACE m_axi     port=input     offset=slave bundle=gmem0 depth=4194304 max_read_burst_length=64
#pragma HLS INTERFACE m_axi     port=gx_output offset=slave bundle=gmem1 depth=4194304 max_write_burst_length=64
#pragma HLS INTERFACE m_axi     port=gy_output offset=slave bundle=gmem2 depth=4194304 max_write_burst_length=64

#pragma HLS INTERFACE s_axilite port=input        bundle=control
#pragma HLS INTERFACE s_axilite port=gx_output    bundle=control
#pragma HLS INTERFACE s_axilite port=gy_output    bundle=control
#pragma HLS INTERFACE s_axilite port=input_height bundle=control
#pragma HLS INTERFACE s_axilite port=input_width  bundle=control
#pragma HLS INTERFACE s_axilite port=return       bundle=control

    if(!is_valid_sobel_config(input_height, input_width))
        return;

    data_t_u32 linebuf0[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS];
    data_t_u32 linebuf1[CONV_INPUT_TILE_ROWS][CONV_LINEBUF_COLS];

#pragma HLS ARRAY_PARTITION variable=linebuf0 complete dim=1
#pragma HLS ARRAY_PARTITION variable=linebuf1 complete dim=1

    load_linebuf_rows_8x8(
        input,
        linebuf0,
        input_height,
        input_width,
        0
    );

outer_y:
    for(int out_y = 0; out_y < input_height; out_y += CONV_PE_ROWS)
    {
        int next_y = out_y + CONV_PE_ROWS;
        bool has_next = (next_y < input_height);

        if(((out_y / CONV_PE_ROWS) & 1) == 0)
        {
            prefetch_and_compute_sobel_conv_rowblock_8x8(
                input,
                gx_output,
                gy_output,
                linebuf0,
                linebuf1,
                input_height,
                input_width,
                out_y,
                next_y,
                has_next
            );
        }
        else
        {
            prefetch_and_compute_sobel_conv_rowblock_8x8(
                input,
                gx_output,
                gy_output,
                linebuf1,
                linebuf0,
                input_height,
                input_width,
                out_y,
                next_y,
                has_next
            );
        }
    }
}
