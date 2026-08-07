
# Builds both blink files as separate targets


# make sure to copy the SDK import into the folder:

# cp $PICO_SDK_PATH/external/pico_sdk_import.cmake ~/firmware/pico/blink/


# check if $PICO_SDK_PATH is set in the shell:

# echo $PICO_SDK_PATH

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(pico_blink C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(blink_left blink_left.c)
pico_add_extra_outputs(blink_left)
target_link_libraries(blink_left pico_stdlib)

add_executable(blink_right blink_right.c)
pico_add_extra_outputs(blink_right)
target_link_libraries(blink_right pico_stdlib)
