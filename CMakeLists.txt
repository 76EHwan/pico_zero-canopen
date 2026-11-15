# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
# RP2350-Zero: waveshare_rp2350_zero / RP2040-Zero: waveshare_rp2040_zero
# (보드를 바꾼 뒤에는 build 폴더를 지우고 다시 configure 하세요)
set(PICO_BOARD waveshare_rp2350_zero CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_zero-canopen C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_zero-canopen
        pico_zero-canopen.c
        src/ak_proto.c
        src/can_bus.c
        src/motor_ctrl.c
        src/serial_cmd.c
        src/usb_io.c
        lib/can2040/src/can2040.c
)

pico_set_program_name(pico_zero-canopen "pico_zero-canopen")
pico_set_program_version(pico_zero-canopen "1.0.0")

# can2040 은 인터럽트 지연에 민감 -> 플래시 캐시 미스가 없도록 전체를 RAM 에서 실행
pico_set_binary_type(pico_zero-canopen copy_to_ram)
set_source_files_properties(lib/can2040/src/can2040.c PROPERTIES COMPILE_OPTIONS "-O2")
set_source_files_properties(
        pico_zero-canopen.c src/ak_proto.c src/can_bus.c src/motor_ctrl.c src/serial_cmd.c src/usb_io.c
        PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")

# 중앙 제어부와는 USB CDC 시리얼로 통신 (UART stdio 는 사용 안 함)
pico_enable_stdio_uart(pico_zero-canopen 0)
pico_enable_stdio_usb(pico_zero-canopen 1)

target_compile_definitions(pico_zero-canopen PRIVATE
        # USB CDC 버퍼 확대 (기본 64 바이트) - 텔레메트리 3줄이 한 번에 들어가도록
        CFG_TUD_CDC_RX_BUFSIZE=1024
        CFG_TUD_CDC_TX_BUFSIZE=1024
        # 호스트가 읽지 않을 때 printf 가 오래 막히지 않도록 (기본 500 ms)
        PICO_STDIO_USB_STDOUT_TIMEOUT_US=5000
)

target_link_libraries(pico_zero-canopen
        pico_stdlib
        pico_multicore
        pico_status_led
        hardware_pio
        hardware_irq
        hardware_clocks
        hardware_watchdog
)

target_include_directories(pico_zero-canopen PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
        ${CMAKE_CURRENT_LIST_DIR}/lib/can2040/src
)

pico_add_extra_outputs(pico_zero-canopen)

