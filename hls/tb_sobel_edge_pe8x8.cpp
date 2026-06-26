#include <iostream>
#include <iomanip>
#include <vector>
#include <chrono>
#include <stdint.h>
#include <limits>
#include "sobel_edge_pe8x8.hpp"

#ifndef TB_INPUT_HEIGHT
#define TB_INPUT_HEIGHT 1024
#endif

#ifndef TB_INPUT_WIDTH
#define TB_INPUT_WIDTH 1024
#endif

static uint32_t xorshift32_state = 123456789u;

static uint32_t rand_u32_full()
{
    uint32_t x = xorshift32_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    xorshift32_state = x;
    return x;
}

static uint32_t read_input_same_padding_tb(
    const std::vector<data_t_u32> &input,
    int height,
    int width,
    int padded_y,
    int padded_x
)
{
    int y = padded_y - CONV_MAX_PADDING;
    int x = padded_x - CONV_MAX_PADDING;

    if(y < 0 || y >= height || x < 0 || x >= width)
        return 0;

    return input[(size_t)y * width + x] & 0xFFu;
}

static void make_sobel_conv_golden(
    const std::vector<data_t_u32> &input,
    std::vector<sobel_conv_t> &gx_golden,
    std::vector<sobel_conv_t> &gy_golden,
    int height,
    int width
)
{
    for(int y = 0; y < height; y++)
    {
        for(int x = 0; x < width; x++)
        {
            int32_t p00 = (int32_t)read_input_same_padding_tb(input, height, width, y + 0, x + 0);
            int32_t p01 = (int32_t)read_input_same_padding_tb(input, height, width, y + 0, x + 1);
            int32_t p02 = (int32_t)read_input_same_padding_tb(input, height, width, y + 0, x + 2);
            int32_t p10 = (int32_t)read_input_same_padding_tb(input, height, width, y + 1, x + 0);
            int32_t p12 = (int32_t)read_input_same_padding_tb(input, height, width, y + 1, x + 2);
            int32_t p20 = (int32_t)read_input_same_padding_tb(input, height, width, y + 2, x + 0);
            int32_t p21 = (int32_t)read_input_same_padding_tb(input, height, width, y + 2, x + 1);
            int32_t p22 = (int32_t)read_input_same_padding_tb(input, height, width, y + 2, x + 2);

            gx_golden[(size_t)y * width + x] = (p02 + (p12 << 1) + p22) - (p00 + (p10 << 1) + p20);
            gy_golden[(size_t)y * width + x] = (p20 + (p21 << 1) + p22) - (p00 + (p01 << 1) + p02);
        }
    }
}

static void print_preview(
    const std::vector<sobel_conv_t> &output,
    const char *name,
    int width,
    int preview_h,
    int preview_w
)
{
    std::cout << name << " top-left " << preview_h << "x" << preview_w << std::endl;
    for(int y = 0; y < preview_h; y++)
    {
        for(int x = 0; x < preview_w; x++)
        {
            std::cout << std::setw(6) << (int)output[(size_t)y * width + x] << ' ';
        }
        std::cout << std::endl;
    }
}

int main()
{
    const int input_height = TB_INPUT_HEIGHT;
    const int input_width = TB_INPUT_WIDTH;
    const int preview_h = 4;
    const int preview_w = 4;

    if(input_height < CONV_MIN_INPUT_HEIGHT || input_width < CONV_MIN_INPUT_WIDTH ||
       input_height > CONV_MAX_INPUT_HEIGHT || input_width > CONV_MAX_INPUT_WIDTH)
    {
        std::cout << "Invalid test configuration" << std::endl;
        return 1;
    }

    std::vector<data_t_u32> input((size_t)input_height * input_width, 0);
    std::vector<sobel_conv_t> gx_hw((size_t)input_height * input_width, 0);
    std::vector<sobel_conv_t> gy_hw((size_t)input_height * input_width, 0);
    std::vector<sobel_conv_t> gx_golden((size_t)input_height * input_width, 0);
    std::vector<sobel_conv_t> gy_golden((size_t)input_height * input_width, 0);

    for(size_t i = 0; i < input.size(); i++)
        input[i] = rand_u32_full() & 0xFFu;

    auto hw_start = std::chrono::high_resolution_clock::now();
    sobel_edge_pe8x8(input.data(), gx_hw.data(), gy_hw.data(), input_height, input_width);
    auto hw_end = std::chrono::high_resolution_clock::now();

    auto golden_start = std::chrono::high_resolution_clock::now();
    make_sobel_conv_golden(input, gx_golden, gy_golden, input_height, input_width);
    auto golden_end = std::chrono::high_resolution_clock::now();

    bool pass = true;
    int mismatch_count = 0;

    for(size_t i = 0; i < gx_hw.size(); i++)
    {
        if(gx_hw[i] != gx_golden[i] || gy_hw[i] != gy_golden[i])
        {
            pass = false;
            if(mismatch_count < 20)
            {
                int y = (int)(i / input_width);
                int x = (int)(i % input_width);
                std::cout << "Mismatch y=" << y
                          << " x=" << x
                          << " gx_hw=" << (int)gx_hw[i]
                          << " gx_golden=" << (int)gx_golden[i]
                          << " gy_hw=" << (int)gy_hw[i]
                          << " gy_golden=" << (int)gy_golden[i]
                          << std::endl;
            }
            mismatch_count++;
        }
    }

    auto hw_us = std::chrono::duration_cast<std::chrono::microseconds>(hw_end - hw_start).count();
    auto golden_us = std::chrono::duration_cast<std::chrono::microseconds>(golden_end - golden_start).count();

    std::cout << "====================================" << std::endl;
    std::cout << "Sobel X/Y Conv-only PE8x8 Test" << std::endl;
    std::cout << "Input size       : " << input_width << "x" << input_height << std::endl;
    std::cout << "Output size      : " << input_width << "x" << input_height << std::endl;
    std::cout << "HW operation     : Sobel X/Y 3x3 convolution only" << std::endl;
    std::cout << "Excluded from HW : abs, magnitude, clip, edge image generation" << std::endl;
    std::cout << "Output type      : int32_t gx, int32_t gy" << std::endl;
    std::cout << "PE array         : " << CONV_PE_ROWS << "x" << CONV_PE_COLS << std::endl;
    std::cout << "HW func time     : " << hw_us << " us" << std::endl;
    std::cout << "Golden time      : " << golden_us << " us" << std::endl;
    std::cout << "====================================" << std::endl;

    if(pass)
        std::cout << "PASS" << std::endl;
    else
        std::cout << "FAIL, mismatch count = " << mismatch_count << std::endl;

    std::cout << std::endl;
    print_preview(gx_hw, "gx_hw", input_width, preview_h, preview_w);
    std::cout << std::endl;
    print_preview(gy_hw, "gy_hw", input_width, preview_h, preview_w);

    std::cout << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();

    return pass ? 0 : 1;
}
