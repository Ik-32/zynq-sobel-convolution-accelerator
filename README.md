# Zynq Sobel Convolution Accelerator

This repository stores the source code for a Zynq-based Sobel 3x3 convolution
accelerator experiment.

The project compares:

- ARM Cortex-A9 software convolution running from a Vitis IDE application.
- A Vitis HLS generated Sobel convolution accelerator integrated through Vivado.

The current code focuses on Sobel X/Y convolution. Edge magnitude generation and
PNG I/O remain in software.

## Repository Layout

```text
hls/
  conv3x3_pe8x8.hpp
  sobel_edge_pe8x8.cpp
  sobel_edge_pe8x8.hpp
  tb_sobel_edge_pe8x8.cpp

vitis-ide/
  main.c
  sobel_edge_accel.c
  sobel_edge_accel.h
```

## HLS Accelerator

Top function:

```cpp
extern "C" void sobel_edge_pe8x8(
    const data_t_u32 *input,
    sobel_conv_t *gx_output,
    sobel_conv_t *gy_output,
    int input_height,
    int input_width
);
```

Main design points:

- 3x3 Sobel convolution with same padding.
- 8x8 PE tile structure.
- Maximum image size is configured as 2048 x 2048.
- AXI master ports are used for input, gx output, and gy output buffers.
- AXI-Lite control registers are used for pointer and image-size arguments.

The HLS testbench is `hls/tb_sobel_edge_pe8x8.cpp`.

## Vitis IDE Application

The bare-metal ARM application:

- Mounts the SD card.
- Reads `0:/1.png` through `0:/100.png`.
- Runs a CPU-only Sobel convolution baseline.
- Runs the HLS accelerator in a blocking measurement mode.
- Runs the HLS accelerator in a pipelined total-time measurement mode.
- Writes output files such as `0:/edge1.png` through `0:/edge100.png`.

The application prints a batch summary with:

- `Convolution only CPU`
- `Convolution with HW`
- `Edge detecting only CPU`
- `Edge detecting with HW`
- `Total only CPU`
- `Total with HW`

## Build Notes

Expected tool flow:

1. Use Vitis HLS to synthesize `sobel_edge_pe8x8`.
2. Export the HLS IP.
3. Add the exported IP to a Vivado block design.
4. Connect AXI master ports to DDR through the Zynq PS interconnect.
5. Connect the AXI-Lite control interface to the Zynq PS.
6. Export the hardware platform to Vitis IDE.
7. Build the ARM Cortex-A9 bare-metal application with the files in `vitis-ide/`.

The Vitis IDE application also depends on the board support package headers and
the PNG library used by the original project, such as `lodepng.h`.

## What Is Stored Here

This repository intentionally stores source code and project notes only. Vivado,
Vitis HLS, and Vitis IDE generated outputs are excluded because they are large
and can be regenerated from the source and tool projects.
