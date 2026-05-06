# Plan: Add USB CDC via TinyUSB to R20_PCBM

## Context

The R20_PCBM is a BLDC motor controller (STM32F042F6P6, 32KB flash, 6KB RAM) that currently has the USB peripheral initialized via HAL PCD but unused. Adding USB CDC (virtual COM port) via TinyUSB enables telemetry and command interfaces over USB. The primary challenge is that this MCU is extremely resource-constrained: the firmware currently uses 27KB of flash at -O0. Switching to -Os and removing the unused HAL USB layer are prerequisites.

## Resource Budget

### Current state (debug, -O0)

| Resource | Used | Total | Free |
|----------|------|-------|------|
| Flash (text) | 27,036 B | 32,768 B | 5,732 B |
| RAM (bss+stack+heap) | ~4,256 B | 6,144 B | ~1,888 B |

### Projected state (with TinyUSB CDC, -Os)

| Resource | Estimated | Total | Estimated Free |
|----------|-----------|-------|----------------|
| Flash | ~17,000-21,500 B | 32,768 B | ~11,000-15,000 B |
| RAM | ~3,700-3,800 B | 6,144 B | ~2,350-2,400 B |

Net RAM actually improves because removing `hpcd_USB_FS` (732B) saves more than TinyUSB state + FIFOs (~250B).

---

## Implementation Steps

### Step 1: Switch from -O0 to -Os

Change the optimization level in both `subdir.mk` build rules to reclaim flash:

**Files to modify:**
- `Debug/Core/Src/subdir.mk` (line 43): `-O0` -> `-Os`
- `Debug/Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk` (line 76): `-O0` -> `-Os`

Expected flash reduction: ~27KB -> ~13-16KB (40-50% savings from HAL code).

### Step 2: Remove HAL PCD, LL_USB, and unused I2C

Since TinyUSB provides its own USB driver (fsdev), the HAL PCD layer becomes dead code.

**2a. Remove from build** (`Debug/Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk`):
- Remove `stm32f0xx_hal_pcd.c`, `stm32f0xx_hal_pcd_ex.c`, `stm32f0xx_ll_usb.c` from C_SRCS/OBJS/C_DEPS
- Remove `stm32f0xx_hal_i2c.c`, `stm32f0xx_hal_i2c_ex.c` (never used)

**2b. Remove bootloader.c** from `Debug/Core/Src/subdir.mk` (file does not exist on disk).

**2c. Disable HAL modules** in `Core/Inc/stm32f0xx_hal_conf.h`:
- Comment out `#define HAL_PCD_MODULE_ENABLED` (line 59)
- Comment out `#define HAL_I2C_MODULE_ENABLED` (line 67)

**2d. Remove PCD MSP functions** in `Core/Src/stm32f0xx_hal_msp.c`:
- Delete `HAL_PCD_MspInit()` (lines 280-295)
- Delete `HAL_PCD_MspDeInit()` (lines 303-317)
- KEEP `HAL_MspInit()` with `__HAL_REMAP_PIN_ENABLE(HAL_REMAP_PA11_PA12)` (critical for TSSOP20 USB pins)

**2e. Remove PCD code from main.c:**
- Delete `PCD_HandleTypeDef hpcd_USB_FS;` (line 52), saves 732B RAM
- Delete `MX_USB_PCD_Init()` prototype, call, and function body

### Step 3: Add TinyUSB source files

Clone/download TinyUSB and place minimal source under `Middlewares/TinyUSB/src/`. Only 6 `.c` files need compilation:

```
Middlewares/TinyUSB/src/
  tusb.c                                      (core)
  common/tusb_fifo.c                          (FIFO)
  device/usbd.c                               (device stack)
  device/usbd_control.c                       (control transfers)
  class/cdc/cdc_device.c                      (CDC class)
  portable/st/stm32_fsdev/dcd_stm32_fsdev.c   (STM32 FS device driver)
```

Plus all required headers from the same directories.

### Step 4: Create tusb_config.h

**New file:** `Core/Inc/tusb_config.h`

Key configuration values:

```c
#define CFG_TUSB_MCU            OPT_MCU_STM32F0
#define BOARD_TUD_RHPORT        0
#define CFG_TUD_ENABLED         1
#define CFG_TUH_ENABLED         0       // device only
#define CFG_TUSB_OS             OPT_OS_NONE
#define CFG_TUSB_DEBUG          0       // saves flash
#define CFG_TUD_ENDPOINT0_SIZE  64
#define CFG_TUD_CDC             1
#define CFG_TUD_MSC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_VENDOR          0
#define CFG_TUD_CDC_RX_BUFSIZE  64      // minimal FIFO
#define CFG_TUD_CDC_TX_BUFSIZE  64      // minimal FIFO
#define CFG_TUD_CDC_EP_BUFSIZE  64
```

### Step 5: Create USB descriptors

**New file:** `Core/Src/usb_descriptors.c`

Implements the three TinyUSB descriptor callbacks:
- `tud_descriptor_device_cb()`: device descriptor (VID/PID, CDC class via IAD)
- `tud_descriptor_configuration_cb()`: config + CDC descriptors using `TUD_CDC_DESCRIPTOR` macro
- `tud_descriptor_string_cb()`: manufacturer ("R20"), product ("PCBM"), serial strings (keep short)

CDC endpoint layout:
- EP1 IN: notification (8B packet size)
- EP2 OUT/IN: data (64B packet size)

PMA allocation: ~312 bytes of 1024 bytes available, fits comfortably.

### Step 6: Wire USB interrupt

**File:** `Core/Src/stm32f0xx_it.c`

```c
#include "tusb.h"

void USB_IRQHandler(void) {
    tud_int_handler(0);
}
```

The weak symbol in `startup_stm32f042f6px.s:305` is overridden.

USB interrupt priority set to 2 (lower than motor control at priority 0). Cortex-M0 has 4 levels (0-3).

### Step 7: Modify main.c

- Add `#include "tusb.h"`
- Add `USB_Init()` function:

```c
static void USB_Init(void) {
    __HAL_RCC_USB_CLK_ENABLE();
    HAL_NVIC_SetPriority(USB_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USB_IRQn);
    tud_init(BOARD_TUD_RHPORT);
}
```

- Replace `MX_USB_PCD_Init()` call with `USB_Init()`
- Add `tud_task()` at the top of the `while(1)` main loop

**HAL_Delay(10000) issue:** During this blocking delay, `tud_task()` is not called so USB enumeration will fail. Solutions:
1. Move `USB_Init()` to after the delay (simplest)
2. Replace the delay with a loop that calls `tud_task()` while waiting:
   ```c
   uint32_t delay_start = HAL_GetTick();
   while (HAL_GetTick() - delay_start < 10000) {
       tud_task();
   }
   ```

### Step 8: Update build system

- Create `Debug/Middlewares/TinyUSB/subdir.mk` with compile rules for the 6 TinyUSB .c files (use `-Os`)
- Add `-I../Middlewares/TinyUSB/src` to include paths in `Debug/Core/Src/subdir.mk`
- Add `usb_descriptors.c` to `Debug/Core/Src/subdir.mk` sources
- Include TinyUSB subdir.mk in `Debug/makefile`
- Add `Middlewares/TinyUSB` to `Debug/sources.mk` SUBDIRS

### Step 9 (optional): Reduce heap

`STM32F042F6PX_FLASH.ld` line 41: `_Min_Heap_Size = 0x200` -> `0x0`

No malloc is used in the application or TinyUSB. Saves 512 bytes of reserved RAM.

---

## Key Constraints

- **PA11/PA12 pin remap** (`HAL_REMAP_PA11_PA12` in `HAL_MspInit`) must be preserved. TinyUSB does not handle this. The remap is critical for the TSSOP20 package.
- **USB priority must be lower** than EXTI (hall sensors) and TIM3 (PID loop) to not disrupt motor control.
- **snprintf with %f** pulls ~3-5KB of float formatting from newlib. Use integer-only formatting for telemetry.
- **TinyUSB fsdev driver** accesses USB registers directly and uses the 1024B PMA for endpoint buffers (not SRAM).
- **USB GPIO**: PA11/PA12 are taken over automatically by the USB peripheral when its clock is enabled. No manual GPIO AF configuration needed.

## Verification

1. Clean build, check with `arm-none-eabi-size`: text < 22KB, bss < 2.5KB
2. Flash firmware via SWD
3. Connect USB: host should enumerate a CDC ACM device (new COM port on Windows, `/dev/ttyACM0` on Linux)
4. Open serial terminal at any baud rate (CDC ignores baud rate) to confirm connection
5. Add telemetry output (e.g. at 10Hz: hall count, speed) and verify data appears on host
6. Verify motor control is not disrupted during USB traffic
