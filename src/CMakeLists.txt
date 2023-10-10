add_library(common common.c)
target_sources(tusb PRIVATE usb_descriptors.c)

add_executable(adc_read adc_read.c)
target_link_libraries(adc_read startupf401xc common freertos tusb printf ${CRT0_OBJ} ${CRT_OBJ})
generate_object(adc_read .hex ihex)

add_executable(blink blink.c)
target_link_libraries(blink startupf401xc common freertos ${CRT0_OBJ} ${CRT_OBJ})
generate_object(blink .hex ihex)

add_executable(hello_usb hello_usb.c)
target_link_libraries(hello_usb startupf401xc common freertos tusb ${CRT0_OBJ} ${CRT_OBJ})
generate_object(hello_usb .hex ihex)

set_target_properties(
    common
    blink
    adc_read
    PROPERTIES
    C_CLANG_TIDY "clang-tidy"
)
