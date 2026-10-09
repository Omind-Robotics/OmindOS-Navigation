# Third-party provenance

The independent target does not replace any upstream license.

- STM32H723 vector order derives from STMicroelectronics' vendor-packaged
  `MDK-ARM/startup_stm32h723xx.s`, Copyright (c) 2019 STMicroelectronics.
- Extracted SystemClock_Config and generated FDCAN/TIM23/USB configuration
  derive from the pinned DM-MC02 Cube-generated project, Copyright (c) 2024
  STMicroelectronics. Upstream copyright notices and component terms apply.
- HAL/USB/ST CMSIS device sources use the license files supplied alongside
  the fixed vendor project. Generic CMSIS uses its supplied Apache-2.0 terms.
- Official RNG source/header additions retain their BSD-3-Clause license.
- FreeRTOS kernel and GCC CM7 port retain their supplied MIT license.
- newlib/libgcc are linked through ARM GCC with their applicable runtime terms.

Exact source commits, build-input hashes and selected source paths are recorded
by the build script. The downloadable candidate contains copies of the
upstream component notices and license files under `licenses/`.
