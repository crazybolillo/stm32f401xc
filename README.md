# stm32f401xc
This is a collection of programs using a STM32F401CCU6 chip. Most if not all of them follow a similar
structure:

* FreeRTOS is used to schedule tasks.
* TinyUSB is used for the USB stack.
* Most data is displayed through a USB CDC connection (can be seen with a serial monitor).
* Whenever possible, only CMSIS5 is used to configure/use the hardware registers. USB is an exception since I value my
  mental sanity.

Overall this project should be of help in the following cases:

* Need some simple examples on how to use FreeRTOS in "real life".
* Want to see how the TinyUSB stack is used for serial-like communication.
* Need an example of how to set up some hardware modules with pure register access (no HAL).

I should mention at this point that I am not an authoritative source of information. I personally enjoy the way I have
set up all code in this project (using clang-format and clang-tidy), but who knows, maybe it's all crap.

## Building
This project uses CMake and assumes you have a working ARM toolchain setup on your computer. The build process
has only been tested (at least that I know of) on Linux systems (specifically Arch Linux btw). You need to use
one of the toolchain files provided in the `vendor/arm-cmake` folder. For example, to build all programs in this
repository, use:

```shell
cmake -DCMAKE_TOOLCHAIN_FILE="vendor/arm-cmake/clang-arm-gcc-toolchain.cmake" -S./ -B./build
cmake --build build
```
