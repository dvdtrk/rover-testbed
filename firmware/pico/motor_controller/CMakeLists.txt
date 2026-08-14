cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK	
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(motor_controller C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

# Loop builds this whole block twice, once for LEFT, once for RIGHT
foreach(SIDE LEFT RIGHT)
	string(TOLOWER ${SIDE} SIDE_LOWER)
	set(TARGET_NAME motor_controller_${SIDE_LOWER})  # ex) motor_controller_left

	add_executable(${TARGET_NAME}
		motor_controller.c
		$ENV{HOME}/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c
	)

	# Defines LEFT_SIDE or RIGHT_SIDE. This is what the #ifdef in motor_controller.c checks to pick node/topic names
	target_compile_definitions(${TARGET_NAME} PRIVATE ${SIDE}_SIDE)

	target_link_libraries(${TARGET_NAME}
		pico_stdlib
		hardware_pwm
		hardware_adc
		hardware_gpio
		hardware_sync
		$ENV{HOME}/micro_ros_raspberrypi_pico_sdk/libmicroros/libmicroros.a
	)

	target_include_directories(${TARGET_NAME} PUBLIC
		$ENV{HOME}/micro_ros_raspberrypi_pico_sdk/libmicroros/include
		$ENV{HOME}/micro_ros_raspberrypi_pico_sdk
	)

	pico_enable_stdio_usb(${TARGET_NAME} 0)
	pico_enable_stdio_uart(${TARGET_NAME} 1)
	pico_add_extra_outputs(${TARGET_NAME})
endforeach()
