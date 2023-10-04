cmake_minimum_required(VERSION 3.26)
project(stm32f4 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(LINKER_FILE "${CMAKE_CURRENT_LIST_DIR}/stm32f401xc.ld")
set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(CPU_FLAGS -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -nostdlib)
set(COMPILER_FLAGS -ffunction-sections -fdata-sections)
add_compile_options(${CPU_FLAGS} ${COMPILER_FLAGS})

include(vendor/arm-cmake/utils.cmake)
if("${CMAKE_C_COMPILER_ID}" STREQUAL "Clang")
    include(vendor/arm-cmake/clang_utils.cmake)
    clang_utils_get_arm_gcc_sysroot(${ARM_GCC_C_COMPILER} ${CPU_FLAGS})
    clang_utils_get_arm_gcc_multilib_dir(${ARM_GCC_C_COMPILER} ${CPU_FLAGS})
    clang_utils_get_arm_gcc_lib_dir(${ARM_GCC_C_COMPILER} ${CPU_FLAGS})
    clang_utils_get_arm_gcc_crt(${ARM_GCC_SYSROOT} ${ARM_GCC_MULTIDIR} ${ARM_GCC_LIBGCC_DIR})
    string(APPEND CMAKE_EXE_LINKER_FLAGS
            " -nostdlib -L${ARM_GCC_SYSROOT}/lib/${ARM_GCC_MULTIDIR} -L${ARM_GCC_LIBGCC_DIR}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " -lgcc -lnosys -lc")
elseif("${CMAKE_C_COMPILER_ID}" STREQUAL "GNU")
    string(REGEX MATCH ".*\.specs.*" has_specs "${CMAKE_EXE_LINKER_FLAGS}")

    if(NOT has_specs)
        string(APPEND CMAKE_EXE_LINKER_FLAGS " --specs=nosys.specs")
    endif()
endif()

string(APPEND CMAKE_EXE_LINKER_FLAGS " -T ${LINKER_FILE} -Wl,--gc-sections")
add_compile_definitions("STM32F401xC")

include_directories(config)
include_directories(vendor/cmsisf4/Include)
include_directories(vendor/freertos/include)
include_directories(vendor/cmsis5/CMSIS/Core/Include)
include_directories(vendor/freertos/portable/GCC/ARM_CM4F)

add_subdirectory(vendor)
add_subdirectory(src)
