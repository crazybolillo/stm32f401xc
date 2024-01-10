add_library(common common.c)
target_sources(tusb PRIVATE usb_descriptors.c)

add_library(base INTERFACE)
target_link_libraries(base INTERFACE
    common
    startupf401xc
    ${CRT0_OBJ}
    ${CRT_OBJ}
)

function(stm32f4_program name)
    add_executable(${name} ${name}.c)
    generate_object(${name} .hex ihex)
endfunction()

stm32f4_program(adc_read)
target_link_libraries(adc_read base freertos tusb printf)

stm32f4_program(blink)
target_link_libraries(blink base freertos)

stm32f4_program(hello_usb)
target_link_libraries(hello_usb base freertos tusb)

stm32f4_program(uart_monitor)
target_link_libraries(uart_monitor base freertos tusb)

stm32f4_program(cny70)
target_link_libraries(cny70 base freertos tusb printf)

stm32f4_program(sharp_ir)
target_link_libraries(sharp_ir base freertos tusb printf m)

stm32f4_program(ultrasonic)
target_link_libraries(ultrasonic base freertos tusb printf)

stm32f4_program(max6675)
target_link_libraries(max6675 base freertos tusb printf)

stm32f4_program(ttp229b)
target_link_libraries(ttp229b base freertos tusb printf)

stm32f4_program(led1642)
target_link_libraries(led1642 base freertos tusb)

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
    led1642
    PROPERTIES
    C_CLANG_TIDY "clang-tidy"
)
