# cmake/wch_toolchain.cmake

# Define target platform
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR riscv)

# Set compiler
set(CMAKE_C_COMPILER riscv-none-embed-gcc)
set(CMAKE_CXX_COMPILER riscv-none-embed-g++)

# Optionally set assembler and other tools
set(CMAKE_ASM_COMPILER riscv-none-embed-gcc)

# Don't look for standard system paths
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
