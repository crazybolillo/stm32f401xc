add_executable(adc_read adc_read.c)
target_link_libraries(adc_read startupf401xc ${CRT0_OBJ} ${CRT_OBJ})
