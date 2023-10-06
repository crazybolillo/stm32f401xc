add_library(common common.c)

add_executable(adc_read adc_read.c)
target_link_libraries(adc_read startupf401xc ${CRT0_OBJ} ${CRT_OBJ})

add_executable(blink blink.c)
target_link_libraries(blink startupf401xc common freertos ${CRT0_OBJ} ${CRT_OBJ})
generate_object(blink .hex ihex)

set_target_properties(
    common
    blink
    PROPERTIES
    C_CLANG_TIDY "clang-tidy"
)
