add_compile_definitions("PRINTF_INCLUDE_CONFIG_H=1")

add_library(
    startupf401xc
    cmsisf4/Source/Templates/gcc/startup_stm32f401xc.s
    cmsisf4/Source/Templates/system_stm32f4xx.c
)
add_library(
    freertos
    freertos/queue.c
    freertos/tasks.c
    freertos/list.c
    freertos/portable/GCC/ARM_CM4F/port.c
    freertos/portable/MemMang/heap_4.c
)
add_library(
    tusb
    tusb/src/tusb.c
    tusb/src/portable/synopsys/dwc2/dcd_dwc2.c
    tusb/src/common/tusb_fifo.c
    tusb/src/device/usbd.c
    tusb/src/device/usbd_control.c
    tusb/src/class/cdc/cdc_device.c
)
add_library(printf printf/src/printf/printf.c)
