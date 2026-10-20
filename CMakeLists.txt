# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)

# Keep the course's programmer choice in source control for clean F401RE builds.
if(BOARD MATCHES "^nucleo_f401re($|/)")
  set(BOARD_FLASH_RUNNER stm32cubeprogrammer CACHE STRING "F401RE flash runner")
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lab2_sensors_actuators)

option(MOTOR_CALIBRATION "Build fixed-duty motor calibration firmware" OFF)
option(SERVO_CALIBRATION "Build manual servo calibration firmware" OFF)
option(CAN_LAB "Lab 3 CAN Demo" OFF)

if(MOTOR_CALIBRATION AND SERVO_CALIBRATION)
  message(FATAL_ERROR "Select only one calibration firmware")
endif()

if (NOT CAN_LAB)
  target_sources(app PRIVATE src/motor_driver.c)
endif()
if(SERVO_CALIBRATION)
  target_include_directories(app PRIVATE src tools/servo_calibration)
  target_sources(app PRIVATE src/servo_driver.c tools/servo_calibration/servo_calibration.c)
elseif(MOTOR_CALIBRATION)
  target_include_directories(app PRIVATE src tools/motor_calibration)
  target_sources(app PRIVATE src/encoder.c tools/motor_calibration/motor_calibration.c)
elseif(CAN_LAB)
  target_include_directories(app PRIVATE src_lab3)
  target_sources(app PRIVATE src_lab3/lab3.c src_lab3/can_bus.c src_lab3/zone_state.c
                             src_lab3/heartbeat.c src_lab3/control.c src_lab3/blink.c
                             src_lab3/button.c src_lab3/n2_tasks.c src_lab3/watchdog.c)
  if(DEFINED NODE_ID)
    target_compile_definitions(app PRIVATE NODE_ID=${NODE_ID})
  endif()
else()
  target_sources(app PRIVATE src/encoder.c src/main.c src/motor_control.c src/blinker.c
                             src/status_tx.c src/servo_driver.c src/wheel_buttons.c)
endif()
