# @brief CMake toolchain file for aarch64 (ARM64) Linux cross-compilation
#
# @note 前置条件
#       安装交叉编译工具链:
#         sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
#
#       SDL2（可选，用于 ex_teleop_f710_servoj）:
#         sudo apt install libsdl2-dev:arm64
#         或 cmake -DSDL2_DIR=/path/to/arm64-sdl2/lib/cmake/SDL2 ...
#
# @usage
#   cd examples/cpp
#   cmake --preset linux-arm64
#   cmake --build --preset linux-arm64

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER   aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Debian/Ubuntu 多架构路径，find_package/find_library 在此搜索 ARM64 库
set(CMAKE_FIND_ROOT_PATH /usr/aarch64-linux-gnu)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
