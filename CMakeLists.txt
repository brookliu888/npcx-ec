# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# Environment variables for building
set(KCONFIG_ROOT ${CMAKE_CURRENT_LIST_DIR}/Kconfig)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR}/platforms)
set(DTS_ROOT ${CMAKE_CURRENT_LIST_DIR})
set(SYSCALL_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include/drivers)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

zephyr_include_directories(include)

# Add the other CMakeLists.txt in sub folders
add_subdirectory(app)
add_subdirectory(drivers)
add_subdirectory(shim)

project(lnv-npcx-ec)

# Compiler options/linker script used by Application
zephyr_compile_options(-Werror -Wno-address-of-packed-member)
zephyr_linker_sources(SECTIONS linker/sections.ld)
