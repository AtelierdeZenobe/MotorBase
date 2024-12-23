# Copyright (c) 2024 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.19.0)

set(MBED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/mbed-os CACHE INTERNAL "")
set(MBED_CONFIG_PATH ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
set(APP_TARGET MotorBase)

include(${MBED_PATH}/tools/cmake/app.cmake)

project(${APP_TARGET})

add_subdirectory(${MBED_PATH})

file(GLOB_RECURSE SRC_FILES "src/*.cpp")
file(GLOB_RECURSE SERVO42C_SRC_FILES "Servo42C_uart/src/*.cpp")
set(ALL_SRC_FILES ${SRC_FILES} ${SERVO42C_SRC_FILES})

add_executable(${APP_TARGET} ${ALL_SRC_FILES})

# Include directories for the application
target_include_directories(${APP_TARGET} PRIVATE
    includes
    Servo42C_uart/includes
)

#add_executable(${APP_TARGET}
#    *.cpp
#)

target_link_libraries(${APP_TARGET} PRIVATE
    mbed-os
    mbed-events
    mbed-storage
)

mbed_set_post_build(${APP_TARGET})

option(VERBOSE_BUILD "Have a verbose build process")
if(VERBOSE_BUILD)
    set(CMAKE_VERBOSE_MAKEFILE ON)
endif()
