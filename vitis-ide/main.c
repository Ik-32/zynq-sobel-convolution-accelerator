#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "xparameters.h"
#include "xil_cache.h"
#include "xil_printf.h"
#include "xil_io.h"
#include "xtime_l.h"
#include "xscugic.h"
#include "xil_exception.h"
#include "xstatus.h"
#include "xil_types.h"
#include "ff.h"

#include "sobel_edge_accel.h"
#include "lodepng.h"

#define GET_US(t) ((uint32_t)((t) / (COUNTS_PER_SECOND / 1000000)))

#define SD_MOUNT_PATH      "0:/"
#define INPUT_FIRST_INDEX  1
#define INPUT_LAST_INDEX   100

#define HLS_MAX_HEIGHT     2048
#define HLS_MAX_WIDTH      2048
#define MAX_PIXELS         (HLS_MAX_HEIGHT * HLS_MAX_WIDTH)

#define HW_SLOT_COUNT      2
#define USE_INTERRUPT      0

/*
 * sobel_edge_pe8x8 top function register map:
 * sobel_edge_pe8x8(input, gx_output, gy_output, input_height, input_width)
 * sobel_edge_accel_start()가 1-output 드라이버여도 main.c에서 직접 gx/gy 레지스터를 씁니다.
 */
#define SOBEL_DIRECT_INPUT_OFFSET       0x10U
#define SOBEL_DIRECT_INPUT_HIGH_OFFSET  0x14U
#define SOBEL_DIRECT_GX_OFFSET          0x1CU
#define SOBEL_DIRECT_GX_HIGH_OFFSET     0x20U
#define SOBEL_DIRECT_GY_OFFSET          0x28U
#define SOBEL_DIRECT_GY_HIGH_OFFSET     0x2CU
#define SOBEL_DIRECT_HEIGHT_OFFSET      0x34U
#define SOBEL_DIRECT_WIDTH_OFFSET       0x3CU

#if defined(XPAR_FABRIC_SOBEL_EDGE_PE8X8_0_INTERRUPT_INTR)
#define SOBEL_ACCEL_INTR_ID XPAR_FABRIC_SOBEL_EDGE_PE8X8_0_INTERRUPT_INTR
#elif defined(XPAR_FABRIC_XSOBEL_EDGE_PE8X8_0_INTERRUPT_INTR)
#define SOBEL_ACCEL_INTR_ID XPAR_FABRIC_XSOBEL_EDGE_PE8X8_0_INTERRUPT_INTR
#else
#define SOBEL_ACCEL_INTR_ID 61u
#endif

#if defined(XPAR_SCUGIC_SINGLE_DEVICE_ID)
#define INTC_DEVICE_ID XPAR_SCUGIC_SINGLE_DEVICE_ID
#elif defined(XPAR_SCUGIC_0_DEVICE_ID)
#define INTC_DEVICE_ID XPAR_SCUGIC_0_DEVICE_ID
#else
#define INTC_DEVICE_ID 0u
#endif

typedef struct {
    unsigned width;
    unsigned height;
    int image_index;
    XTime hw_start_time;
    int running;
} hw_slot_meta_t;

static FATFS fatfs;
static XScuGic intc;
static volatile int sobel_done = 0;

static uint32_t input_buf[HW_SLOT_COUNT][MAX_PIXELS] __attribute__((aligned(64)));
static int32_t hw_gx_buf[HW_SLOT_COUNT][MAX_PIXELS] __attribute__((aligned(64)));
static int32_t hw_gy_buf[HW_SLOT_COUNT][MAX_PIXELS] __attribute__((aligned(64)));
static uint8_t hw_edge_buf[HW_SLOT_COUNT][MAX_PIXELS] __attribute__((aligned(64)));
static hw_slot_meta_t hw_slot[HW_SLOT_COUNT];

static int32_t sw_gx_buf[MAX_PIXELS] __attribute__((aligned(64)));
static int32_t sw_gy_buf[MAX_PIXELS] __attribute__((aligned(64)));
static uint8_t sw_edge_buf[MAX_PIXELS] __attribute__((aligned(64)));

static int setup_interrupt_system(void);
static void sobel_isr(void *callback_ref);
static int sd_mount(void);
static int sd_read_file_to_memory(const char *path, unsigned char **data, size_t *size);
static int sd_write_file_from_memory(const char *path, const unsigned char *data, size_t size);
static int load_png_to_input_slot(int image_index, int slot);
static void rgba_to_hls_gray_input(uint32_t *input, const unsigned char *rgba, unsigned width, unsigned height);
static uint8_t cpu_read_gray_same_padding(const uint32_t *input, int height, int width, int padded_y, int padded_x);
static void cpu_sobel_conv_xy_only(const uint32_t *input, int height, int width, int32_t *gx_out, int32_t *gy_out);
static void cpu_sobel_postprocess_edge(const int32_t *gx, const int32_t *gy, uint8_t *edge, int height, int width);
static int encode_and_write_png(const char *path, const uint8_t *edge, unsigned width, unsigned height);
static void make_output_path(char *path, size_t path_size, int image_index);
static void sobel_write_addr(uint32_t low_offset, uint32_t high_offset, UINTPTR addr);
static void sobel_start_conv_xy(UINTPTR input_ptr, UINTPTR gx_ptr, UINTPTR gy_ptr, int height, int width);
static int start_sobel_conv_hw_slot(int slot);
static int finish_sobel_conv_hw_slot(int slot, uint32_t *conv_hw_us);
static int run_cpu_only_batch(uint64_t *conv_only_cpu_us,
                              uint64_t *edge_detecting_only_cpu_us,
                              uint64_t *total_only_cpu_us,
                              int *processed_count);
static int run_hw_blocking_measure_batch(uint64_t *conv_with_hw_us,
                                         uint64_t *edge_detecting_with_hw_us,
                                         int *processed_count);
static int run_hw_pipeline_batch(uint64_t *total_with_hw_us,
                                 int *processed_count);
static void print_u64_us(const char *label, uint64_t value);
static void print_u64_count(const char *label, uint64_t value);

int main(void)
{
    XTime t0, t1;
    uint32_t sd_mount_us = 0;
    uint64_t conv_only_cpu_us = 0;
    uint64_t conv_with_hw_us = 0;
    uint64_t edge_detecting_only_cpu_us = 0;
    uint64_t edge_detecting_with_hw_us = 0;
    uint64_t total_only_cpu_us = 0;
    uint64_t total_with_hw_us = 0;
    int cpu_processed_count = 0;
    int hw_measure_processed_count = 0;
    int hw_pipeline_processed_count = 0;

    xil_printf("\r\n--- SD PNG Sobel Batch Edge Detection using sobel_edge_pe8x8 HLS IP ---\r\n");
    xil_printf("Input  : 0:/1.png ~ 0:/100.png\r\n");
    xil_printf("Output : 0:/edge1.png ~ 0:/edge100.png\r\n");
    xil_printf("Mode   : CPU-only timing, HW blocking timing, then HW pipelined total timing.\r\n");

    XTime_GetTime(&t0);
    if(sd_mount() != XST_SUCCESS) {
        xil_printf("ERROR: SD mount failed.\r\n");
        return XST_FAILURE;
    }
    XTime_GetTime(&t1);
    sd_mount_us = GET_US(t1 - t0);

    sobel_edge_accel_init(SOBEL_EDGE_ACCEL_BASEADDR);

#if USE_INTERRUPT
    if(setup_interrupt_system() != XST_SUCCESS) {
        xil_printf("ERROR: interrupt setup failed. Set USE_INTERRUPT to 0 and test polling first.\r\n");
        return XST_FAILURE;
    }
#endif

    if(run_cpu_only_batch(&conv_only_cpu_us,
                          &edge_detecting_only_cpu_us,
                          &total_only_cpu_us,
                          &cpu_processed_count) != XST_SUCCESS) {
        xil_printf("ERROR: CPU-only batch failed.\r\n");
        return XST_FAILURE;
    }

    if(run_hw_blocking_measure_batch(&conv_with_hw_us,
                                      &edge_detecting_with_hw_us,
                                      &hw_measure_processed_count) != XST_SUCCESS) {
        xil_printf("ERROR: HW blocking measurement batch failed.\r\n");
        return XST_FAILURE;
    }

    if(run_hw_pipeline_batch(&total_with_hw_us,
                             &hw_pipeline_processed_count) != XST_SUCCESS) {
        xil_printf("ERROR: HW pipeline batch failed.\r\n");
        return XST_FAILURE;
    }

    total_only_cpu_us += sd_mount_us;
    total_with_hw_us += sd_mount_us;

    xil_printf("============= Batch Summary =============\r\n");
    print_u64_count("CPU processed images   : ", (uint64_t)cpu_processed_count);
    print_u64_count("HW measured images     : ", (uint64_t)hw_measure_processed_count);
    print_u64_count("HW output images       : ", (uint64_t)hw_pipeline_processed_count);
    print_u64_us("Convolution only CPU   : ", conv_only_cpu_us);
    print_u64_us("Convolution with HW    : ", conv_with_hw_us);
    print_u64_us("Edge detecting only CPU: ", edge_detecting_only_cpu_us);
    print_u64_us("Edge detecting with HW : ", edge_detecting_with_hw_us);
    print_u64_us("Total only CPU         : ", total_only_cpu_us);
    print_u64_us("Total with HW          : ", total_with_hw_us);
    xil_printf("DONE\r\n");

    return XST_SUCCESS;
}

static int run_cpu_only_batch(uint64_t *conv_only_cpu_us,
                              uint64_t *edge_detecting_only_cpu_us,
                              uint64_t *total_only_cpu_us,
                              int *processed_count)
{
    XTime batch_t0, batch_t1;
    int image_index;

    *conv_only_cpu_us = 0;
    *edge_detecting_only_cpu_us = 0;
    *total_only_cpu_us = 0;
    *processed_count = 0;

    XTime_GetTime(&batch_t0);

    for(image_index = INPUT_FIRST_INDEX; image_index <= INPUT_LAST_INDEX; image_index++) {
        XTime t0, t1;
        XTime edge_t0, edge_t1;
        char output_path[32];
        uint32_t conv_us;
        unsigned width;
        unsigned height;

        if(load_png_to_input_slot(image_index, 0) != XST_SUCCESS)
            return XST_FAILURE;

        width = hw_slot[0].width;
        height = hw_slot[0].height;

        XTime_GetTime(&edge_t0);

        XTime_GetTime(&t0);
        cpu_sobel_conv_xy_only(input_buf[0], (int)height, (int)width, sw_gx_buf, sw_gy_buf);
        XTime_GetTime(&t1);
        conv_us = GET_US(t1 - t0);

        cpu_sobel_postprocess_edge(sw_gx_buf, sw_gy_buf, sw_edge_buf, (int)height, (int)width);

        XTime_GetTime(&edge_t1);

        make_output_path(output_path, sizeof(output_path), image_index);
        if(encode_and_write_png(output_path, sw_edge_buf, width, height) != XST_SUCCESS)
            return XST_FAILURE;

        *conv_only_cpu_us += conv_us;
        *edge_detecting_only_cpu_us += GET_US(edge_t1 - edge_t0);
        (*processed_count)++;

        xil_printf("CPU pass [%d/%d] done\r\n", image_index, INPUT_LAST_INDEX);
    }

    XTime_GetTime(&batch_t1);
    *total_only_cpu_us = GET_US(batch_t1 - batch_t0);

    return XST_SUCCESS;
}

static int run_hw_blocking_measure_batch(uint64_t *conv_with_hw_us,
                                         uint64_t *edge_detecting_with_hw_us,
                                         int *processed_count)
{
    int image_index;

    *conv_with_hw_us = 0;
    *edge_detecting_with_hw_us = 0;
    *processed_count = 0;

    for(image_index = INPUT_FIRST_INDEX; image_index <= INPUT_LAST_INDEX; image_index++) {
        XTime edge_t0, edge_t1;
        uint32_t conv_hw_us = 0;
        unsigned width;
        unsigned height;

        if(load_png_to_input_slot(image_index, 0) != XST_SUCCESS)
            return XST_FAILURE;

        width = hw_slot[0].width;
        height = hw_slot[0].height;
        (void)width;

        XTime_GetTime(&edge_t0);

        if(start_sobel_conv_hw_slot(0) != XST_SUCCESS)
            return XST_FAILURE;

        if(finish_sobel_conv_hw_slot(0, &conv_hw_us) != XST_SUCCESS)
            return XST_FAILURE;

        cpu_sobel_postprocess_edge(hw_gx_buf[0],
                                   hw_gy_buf[0],
                                   hw_edge_buf[0],
                                   (int)height,
                                   (int)hw_slot[0].width);

        XTime_GetTime(&edge_t1);

        *conv_with_hw_us += conv_hw_us;
        *edge_detecting_with_hw_us += GET_US(edge_t1 - edge_t0);
        (*processed_count)++;

        xil_printf("HW measure [%d/%d] done\r\n", image_index, INPUT_LAST_INDEX);
    }

    return XST_SUCCESS;
}

static int run_hw_pipeline_batch(uint64_t *total_with_hw_us,
                                 int *processed_count)
{
    XTime batch_t0, batch_t1;
    int first_slot = 0;
    int running_slot;
    int running_index;
    int image_index;

    *total_with_hw_us = 0;
    *processed_count = 0;

    XTime_GetTime(&batch_t0);

    if(load_png_to_input_slot(INPUT_FIRST_INDEX, first_slot) != XST_SUCCESS)
        return XST_FAILURE;

    if(start_sobel_conv_hw_slot(first_slot) != XST_SUCCESS)
        return XST_FAILURE;

    running_slot = first_slot;
    running_index = INPUT_FIRST_INDEX;

    for(image_index = INPUT_FIRST_INDEX + 1; image_index <= INPUT_LAST_INDEX; image_index++) {
        int next_slot = (image_index - INPUT_FIRST_INDEX) & 1;
        char output_path[32];

        if(load_png_to_input_slot(image_index, next_slot) != XST_SUCCESS) {
            finish_sobel_conv_hw_slot(running_slot, NULL);
            return XST_FAILURE;
        }

        if(finish_sobel_conv_hw_slot(running_slot, NULL) != XST_SUCCESS)
            return XST_FAILURE;

        if(start_sobel_conv_hw_slot(next_slot) != XST_SUCCESS)
            return XST_FAILURE;

        cpu_sobel_postprocess_edge(hw_gx_buf[running_slot],
                                   hw_gy_buf[running_slot],
                                   hw_edge_buf[running_slot],
                                   (int)hw_slot[running_slot].height,
                                   (int)hw_slot[running_slot].width);

        make_output_path(output_path, sizeof(output_path), running_index);
        if(encode_and_write_png(output_path,
                                hw_edge_buf[running_slot],
                                hw_slot[running_slot].width,
                                hw_slot[running_slot].height) != XST_SUCCESS)
            return XST_FAILURE;

        (*processed_count)++;

        xil_printf("HW pipeline [%d/%d] done\r\n", running_index, INPUT_LAST_INDEX);

        running_slot = next_slot;
        running_index = image_index;
    }

    {
        char output_path[32];

        if(finish_sobel_conv_hw_slot(running_slot, NULL) != XST_SUCCESS)
            return XST_FAILURE;

        cpu_sobel_postprocess_edge(hw_gx_buf[running_slot],
                                   hw_gy_buf[running_slot],
                                   hw_edge_buf[running_slot],
                                   (int)hw_slot[running_slot].height,
                                   (int)hw_slot[running_slot].width);

        make_output_path(output_path, sizeof(output_path), running_index);
        if(encode_and_write_png(output_path,
                                hw_edge_buf[running_slot],
                                hw_slot[running_slot].width,
                                hw_slot[running_slot].height) != XST_SUCCESS)
            return XST_FAILURE;

        (*processed_count)++;

        xil_printf("HW pipeline [%d/%d] done\r\n", running_index, INPUT_LAST_INDEX);
    }

    XTime_GetTime(&batch_t1);
    *total_with_hw_us = GET_US(batch_t1 - batch_t0);

    return XST_SUCCESS;
}

static int load_png_to_input_slot(int image_index, int slot)
{
    XTime t0, t1;
    char input_path[32];
    unsigned char *png_input = NULL;
    size_t png_input_size = 0;
    unsigned char *rgba_input = NULL;
    unsigned width = 0;
    unsigned height = 0;
    unsigned lodepng_err = 0;

    if(slot < 0 || slot >= HW_SLOT_COUNT)
        return XST_FAILURE;

    snprintf(input_path, sizeof(input_path), "0:/%d.png", image_index);

    XTime_GetTime(&t0);
    if(sd_read_file_to_memory(input_path, &png_input, &png_input_size) != XST_SUCCESS) {
        xil_printf("ERROR: failed to read %s\r\n", input_path);
        return XST_FAILURE;
    }
    XTime_GetTime(&t1);
    (void)t0;
    (void)t1;

    lodepng_err = lodepng_decode32(&rgba_input, &width, &height, png_input, png_input_size);
    free(png_input);
    png_input = NULL;
    if(lodepng_err != 0u) {
        xil_printf("ERROR: PNG decode failed. file=%s code=%d\r\n", input_path, (int)lodepng_err);
        return XST_FAILURE;
    }

    if(width == 0u || height == 0u || width > HLS_MAX_WIDTH || height > HLS_MAX_HEIGHT) {
        xil_printf("ERROR: invalid image size. file=%s width=%d height=%d\r\n", input_path, (int)width, (int)height);
        free(rgba_input);
        return XST_FAILURE;
    }

    rgba_to_hls_gray_input(input_buf[slot], rgba_input, width, height);
    free(rgba_input);
    rgba_input = NULL;

    hw_slot[slot].width = width;
    hw_slot[slot].height = height;
    hw_slot[slot].image_index = image_index;

    return XST_SUCCESS;
}

static int sd_mount(void)
{
    FRESULT fr = f_mount(&fatfs, SD_MOUNT_PATH, 1);
    if(fr != FR_OK) {
        xil_printf("f_mount failed: %d\r\n", fr);
        return XST_FAILURE;
    }
    return XST_SUCCESS;
}

static int sd_read_file_to_memory(const char *path, unsigned char **data, size_t *size)
{
    FIL file;
    FRESULT fr;
    FSIZE_t file_size;
    UINT br = 0;
    unsigned char *buf;

    *data = NULL;
    *size = 0;

    fr = f_open(&file, path, FA_READ);
    if(fr != FR_OK) {
        xil_printf("f_open read failed. path=%s err=%d\r\n", path, fr);
        return XST_FAILURE;
    }

    file_size = f_size(&file);
    if(file_size == 0u || file_size > 0x7FFFFFFFu) {
        xil_printf("invalid file size. path=%s\r\n", path);
        f_close(&file);
        return XST_FAILURE;
    }

    buf = (unsigned char *)malloc((size_t)file_size);
    if(buf == NULL) {
        xil_printf("malloc failed for input PNG. path=%s\r\n", path);
        f_close(&file);
        return XST_FAILURE;
    }

    fr = f_read(&file, buf, (UINT)file_size, &br);
    f_close(&file);

    if(fr != FR_OK || br != (UINT)file_size) {
        xil_printf("f_read failed. path=%s err=%d read=%d\r\n", path, fr, (int)br);
        free(buf);
        return XST_FAILURE;
    }

    *data = buf;
    *size = (size_t)file_size;
    return XST_SUCCESS;
}

static int sd_write_file_from_memory(const char *path, const unsigned char *data, size_t size)
{
    FIL file;
    FRESULT fr;
    UINT bw = 0;

    fr = f_open(&file, path, FA_CREATE_ALWAYS | FA_WRITE);
    if(fr != FR_OK) {
        xil_printf("f_open write failed. path=%s err=%d\r\n", path, fr);
        return XST_FAILURE;
    }

    fr = f_write(&file, data, (UINT)size, &bw);
    f_close(&file);

    if(fr != FR_OK || bw != (UINT)size) {
        xil_printf("f_write failed. path=%s err=%d written=%d\r\n", path, fr, (int)bw);
        return XST_FAILURE;
    }

    return XST_SUCCESS;
}

static void rgba_to_hls_gray_input(uint32_t *input, const unsigned char *rgba, unsigned width, unsigned height)
{
    unsigned y, x;

    for(y = 0; y < height; y++) {
        for(x = 0; x < width; x++) {
            size_t idx = ((size_t)y * width + x) * 4u;
            uint32_t r = (uint32_t)rgba[idx + 0u];
            uint32_t g = (uint32_t)rgba[idx + 1u];
            uint32_t b = (uint32_t)rgba[idx + 2u];
            uint32_t gray = (77u * r + 150u * g + 29u * b) >> 8;
            input[(size_t)y * width + x] = gray & 0xFFu;
        }
    }
}

static uint8_t cpu_read_gray_same_padding(const uint32_t *input, int height, int width, int padded_y, int padded_x)
{
    int in_y = padded_y - 1;
    int in_x = padded_x - 1;

    if(in_y < 0 || in_y >= height || in_x < 0 || in_x >= width)
        return 0;

    return (uint8_t)(input[(size_t)in_y * width + in_x] & 0xFFu);
}

static void cpu_sobel_conv_xy_only(const uint32_t *input, int height, int width, int32_t *gx_out, int32_t *gy_out)
{
    int y, x;

    for(y = 0; y < height; y++) {
        for(x = 0; x < width; x++) {
            int32_t p00 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 0, x + 0);
            int32_t p01 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 0, x + 1);
            int32_t p02 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 0, x + 2);
            int32_t p10 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 1, x + 0);
            int32_t p12 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 1, x + 2);
            int32_t p20 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 2, x + 0);
            int32_t p21 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 2, x + 1);
            int32_t p22 = (int32_t)cpu_read_gray_same_padding(input, height, width, y + 2, x + 2);
            size_t idx = (size_t)y * width + x;

            gx_out[idx] = (p02 + (p12 << 1) + p22) - (p00 + (p10 << 1) + p20);
            gy_out[idx] = (p20 + (p21 << 1) + p22) - (p00 + (p01 << 1) + p02);
        }
    }
}

static void cpu_sobel_postprocess_edge(const int32_t *gx, const int32_t *gy, uint8_t *edge, int height, int width)
{
    int y, x;

    for(y = 0; y < height; y++) {
        for(x = 0; x < width; x++) {
            size_t idx = (size_t)y * width + x;
            int32_t gx_v = gx[idx];
            int32_t gy_v = gy[idx];
            int32_t mag = ((gx_v < 0) ? -gx_v : gx_v) + ((gy_v < 0) ? -gy_v : gy_v);

            if(mag > 255)
                edge[idx] = 255u;
            else
                edge[idx] = (uint8_t)mag;
        }
    }
}

static int encode_and_write_png(const char *path, const uint8_t *edge, unsigned width, unsigned height)
{
    unsigned char *png_output = NULL;
    size_t png_output_size = 0;
    unsigned lodepng_err;

    lodepng_err = lodepng_encode_memory(&png_output,
                                         &png_output_size,
                                         edge,
                                         width,
                                         height,
                                         LCT_GREY,
                                         8);

    if(lodepng_err != 0u) {
        xil_printf("ERROR: PNG encode failed. path=%s code=%d\r\n", path, (int)lodepng_err);
        return XST_FAILURE;
    }

    if(sd_write_file_from_memory(path, png_output, png_output_size) != XST_SUCCESS) {
        free(png_output);
        return XST_FAILURE;
    }

    free(png_output);
    return XST_SUCCESS;
}

static void make_output_path(char *path, size_t path_size, int image_index)
{
    snprintf(path, path_size, "0:/edge%d.png", image_index);
}

static int start_sobel_conv_hw_slot(int slot)
{
    size_t pixel_count;
    size_t input_bytes;
    size_t output_bytes;

    if(slot < 0 || slot >= HW_SLOT_COUNT)
        return XST_FAILURE;

    pixel_count = (size_t)hw_slot[slot].height * (size_t)hw_slot[slot].width;
    input_bytes = pixel_count * sizeof(input_buf[slot][0]);
    output_bytes = pixel_count * sizeof(hw_gx_buf[slot][0]);

    Xil_DCacheFlushRange((UINTPTR)input_buf[slot], (u32)input_bytes);
    Xil_DCacheFlushRange((UINTPTR)hw_gx_buf[slot], (u32)output_bytes);
    Xil_DCacheFlushRange((UINTPTR)hw_gy_buf[slot], (u32)output_bytes);

#if USE_INTERRUPT
    sobel_done = 0;
    sobel_edge_accel_interrupt_clear();
#endif

    XTime_GetTime(&hw_slot[slot].hw_start_time);

    sobel_start_conv_xy((UINTPTR)input_buf[slot],
                        (UINTPTR)hw_gx_buf[slot],
                        (UINTPTR)hw_gy_buf[slot],
                        (int)hw_slot[slot].height,
                        (int)hw_slot[slot].width);

    hw_slot[slot].running = 1;
    return XST_SUCCESS;
}

static int finish_sobel_conv_hw_slot(int slot, uint32_t *conv_hw_us)
{
    XTime t1;
    size_t pixel_count;
    size_t output_bytes;

    if(slot < 0 || slot >= HW_SLOT_COUNT)
        return XST_FAILURE;

    if(!hw_slot[slot].running)
        return XST_FAILURE;

#if USE_INTERRUPT
    while(!sobel_done) {
        /* busy wait */
    }
#else
    sobel_edge_accel_wait_done();
#endif

    XTime_GetTime(&t1);

    pixel_count = (size_t)hw_slot[slot].height * (size_t)hw_slot[slot].width;
    output_bytes = pixel_count * sizeof(hw_gx_buf[slot][0]);

    Xil_DCacheInvalidateRange((UINTPTR)hw_gx_buf[slot], (u32)output_bytes);
    Xil_DCacheInvalidateRange((UINTPTR)hw_gy_buf[slot], (u32)output_bytes);

    if(conv_hw_us != NULL)
        *conv_hw_us = GET_US(t1 - hw_slot[slot].hw_start_time);

    hw_slot[slot].running = 0;
    return XST_SUCCESS;
}

static void sobel_write_addr(uint32_t low_offset, uint32_t high_offset, UINTPTR addr)
{
    UINTPTR base = sobel_edge_accel_get_baseaddr();
    Xil_Out32(base + low_offset, (uint32_t)((uint64_t)addr & 0xFFFFFFFFu));
    Xil_Out32(base + high_offset, (uint32_t)(((uint64_t)addr >> 32) & 0xFFFFFFFFu));
}

static void sobel_start_conv_xy(UINTPTR input_ptr, UINTPTR gx_ptr, UINTPTR gy_ptr, int height, int width)
{
    UINTPTR base = sobel_edge_accel_get_baseaddr();

    sobel_write_addr(SOBEL_DIRECT_INPUT_OFFSET, SOBEL_DIRECT_INPUT_HIGH_OFFSET, input_ptr);
    sobel_write_addr(SOBEL_DIRECT_GX_OFFSET, SOBEL_DIRECT_GX_HIGH_OFFSET, gx_ptr);
    sobel_write_addr(SOBEL_DIRECT_GY_OFFSET, SOBEL_DIRECT_GY_HIGH_OFFSET, gy_ptr);

    Xil_Out32(base + SOBEL_DIRECT_HEIGHT_OFFSET, (uint32_t)height);
    Xil_Out32(base + SOBEL_DIRECT_WIDTH_OFFSET, (uint32_t)width);
    Xil_Out32(base + SOBEL_EDGE_CTRL_OFFSET, SOBEL_EDGE_AP_START_MASK);
}

static void print_u64_us(const char *label, uint64_t value)
{
    char buf[32];
    int pos = 30;

    buf[31] = '\0';

    if(value == 0u) {
        buf[pos] = '0';
        xil_printf("%s%s us\r\n", label, &buf[pos]);
        return;
    }

    while(value > 0u && pos >= 0) {
        buf[pos] = (char)('0' + (value % 10u));
        value /= 10u;
        pos--;
    }

    xil_printf("%s%s us\r\n", label, &buf[pos + 1]);
}

static void print_u64_count(const char *label, uint64_t value)
{
    char buf[32];
    int pos = 30;

    buf[31] = '\0';

    if(value == 0u) {
        buf[pos] = '0';
        xil_printf("%s%s\r\n", label, &buf[pos]);
        return;
    }

    while(value > 0u && pos >= 0) {
        buf[pos] = (char)('0' + (value % 10u));
        value /= 10u;
        pos--;
    }

    xil_printf("%s%s\r\n", label, &buf[pos + 1]);
}

static int setup_interrupt_system(void)
{
    XScuGic_Config *cfg;
    int status;

    cfg = XScuGic_LookupConfig(INTC_DEVICE_ID);
    if(cfg == NULL)
        return XST_FAILURE;

    status = XScuGic_CfgInitialize(&intc, cfg, cfg->CpuBaseAddress);
    if(status != XST_SUCCESS)
        return XST_FAILURE;

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
                                 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
                                 &intc);

    status = XScuGic_Connect(&intc,
                              SOBEL_ACCEL_INTR_ID,
                              (Xil_ExceptionHandler)sobel_isr,
                              NULL);
    if(status != XST_SUCCESS)
        return XST_FAILURE;

    XScuGic_Enable(&intc, SOBEL_ACCEL_INTR_ID);
    Xil_ExceptionEnable();

    sobel_edge_accel_interrupt_enable();

    return XST_SUCCESS;
}

static void sobel_isr(void *callback_ref)
{
    (void)callback_ref;
    sobel_edge_accel_interrupt_clear();
    sobel_done = 1;
}
