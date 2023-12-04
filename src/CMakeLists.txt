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

add_executable(uart_monitor uart_monitor.c)
target_link_libraries(uart_monitor startupf401xc common freertos tusb ${CRT0_OBJ} ${CRT_OBJ})
generate_object(uart_monitor .hex ihex)

add_executable(cny70 cny70.c)
target_link_libraries(cny70 startupf401xc common freertos tusb printf ${CRT0_OBJ} ${CRT_OBJ})
generate_object(cny70 .hex ihex)

add_executable(sharp_ir sharp_ir.c)
target_link_libraries(sharp_ir startupf401xc common freertos tusb printf m ${CRT0_OBJ} ${CRT_OBJ})
generate_object(sharp_ir .hex ihex)

add_executable(ultrasonic ultrasonic.c)
target_link_libraries(ultrasonic startupf401xc common freertos tusb printf ${CRT0_OBJ} ${CRT_OBJ})
generate_object(ultrasonic .hex ihex)

add_executable(max6675 max6675.c)
target_link_libraries(max6675 startupf401xc common freertos tusb printf ${CRT0_OBJ} ${CRT_OBJ})
generate_object(max6675 .hex ihex)

add_executable(ttp229b ttp229b.c)
target_link_libraries(ttp229b startupf401xc common freertos tusb printf ${CRT0_OBJ} ${CRT_OBJ})
generate_object(ttp229b .hex ihex)

set_target_properties(
    common
    blink
    adc_read
    uart_monitor
    cny70
    sharp_ir
    ultrasonic
    max6675
    ttp229b
    PROPERTIES
    C_CLANG_TIDY "clang-tidy"
)
