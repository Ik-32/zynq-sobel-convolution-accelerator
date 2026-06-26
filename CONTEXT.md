# Project Context

Owner GitHub account: `Ik-32`

Created from local source files on 2026-06-26.

## Purpose

This project is part of an undergraduate embedded systems / FPGA accelerator
study. The goal is to compare software execution on the ARM Cortex-A9 with a
hardware accelerator generated using Vitis HLS and integrated using Vivado.

## Important Memory

- The C++ files are the Vitis HLS accelerator source.
- The C files are the Vitis IDE / ARM Cortex-A9 software application source.
- The accelerator computes Sobel X/Y 3x3 convolution.
- The ARM application performs CPU baseline computation and compares timing
  against the hardware accelerator.
- The tool flow used for this project is Vitis HLS, Vivado, and Vitis IDE.
- The broader research direction is CNN accelerator implementation, object
  detection, and software-vs-hardware power/performance comparison.

## Measurement Meaning

The hardware accelerator timing is not the full image-processing pipeline by
itself. The HLS IP accelerates the convolution stage. PNG I/O and final edge
image generation are handled in software, so performance numbers should be
reported with the measured stage clearly named.

## Next Useful Work

- Add exact board name and Vivado block design details.
- Save HLS synthesis reports.
- Save measured CPU/HW timing results.
- Save power measurement method and board rail/input-power conditions.
