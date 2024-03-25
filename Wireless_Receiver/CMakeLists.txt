cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/pico_sdk_init.cmake)

project(wireless_attempt CXX C ASM)

pico_sdk_init()

include_directories(
	include
	)

add_executable(wireless_attempt
	src/joybus.cpp
	src/main.cpp
	)

pico_generate_pio_header(wireless_attempt ${CMAKE_CURRENT_LIST_DIR}/pio/joybus.pio)

add_custom_command(OUTPUT ${CMAKE_CURRENT_LIST_DIR}/generated/joybus.pio.h
	DEPENDS ${CMAKE_CURRENT_LIST_DIR}/joybus.pio
	COMMAND Pioasm ${CMAKE_CURRENT_LIST_DIR}/joybus.pio ${CMAKE_CURRENT_LIST_DIR}/generated/joybus.pio.h
	)

target_compile_definitions(wireless_attempt
	PRIVATE
	PICO_XOSC_STARTUP_DELAY_MULTIPLIER=64
	#PICO_DEFAULT_BOOT_STAGE2=boot2_generic_03h
	#PICO_FLASH_SPI_CLKDIV=8
	)

target_link_libraries(wireless_attempt
	PRIVATE
	pico_stdlib
	pico_bootrom
	pico_multicore
	pico_platform
	hardware_adc
	hardware_dma
	hardware_flash
	hardware_pio
	hardware_pwm
	hardware_spi
	hardware_sync
	hardware_timer
)

pico_add_extra_outputs(wireless_attempt)
pico_enable_stdio_usb(wireless_attempt 1)
pico_enable_stdio_uart(wireless_attempt 0)

