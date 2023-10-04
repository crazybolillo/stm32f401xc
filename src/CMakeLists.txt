add_executable(adc_read adc_read.c)
target_link_libraries(adc_read startupf401xc ${CRT0_OBJ} ${CRT_OBJ})

add_executable(blink blink.c)
target_link_libraries(blink startupf401xc freertos ${CRT0_OBJ} ${CRT_OBJ})
generate_object(blink .hex ihex)
