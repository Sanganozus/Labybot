# Use "-DCMAKE_TOOLCHAIN_FILE=generic-gcc-avr.cmake" as CMake command line
# option when configuring build

cmake_minimum_required(VERSION 3.18)

### TOOLCHAIN SETUP AREA #################################################
# Set any variables used in the toolchain prior to project() call. In that
# case they are already set and used.
##########################################################################

##########################################################################
# upload tool is not used for HWP, no need to define it
##########################################################################
#set(AVR_UPLOADTOOL avrdude)
#set(AVR_PROGRAMMER avr109)
#set(AVR_UPLOADTOOL_PORT usb)
#set(AVR_UPLOADTOOL_BAUDRATE 5000000)

##########################################################################
# AVR_MCU must be set for identifying the microcontroller
##########################################################################
set(AVR_MCU atmega1280)

##########################################################################
# programming of fuses is not used for HWP
##########################################################################
#set(AVR_E_FUSE 0xfd)
#set(AVR_H_FUSE 0x9c)
#set(AVR_L_FUSE 0xe2)

### END TOOLCHAIN SETUP AREA #############################################


##########################################################################
# name your project and set used programming languages
##########################################################################
project(HWPRobot C ASM)

##########################################################################
# enable verbose Makefile to see full compiler and linker calls
##########################################################################
set(CMAKE_VERBOSE_MAKEFILE ON)

##########################################################################
# clock speed of MCU needs to be defined for some functions from avr-libc
##########################################################################
set(MCU_SPEED "8000000UL")

##########################################################################
# set include and library paths for the AVR toolchain. This is not
# added automatically, since CMAKE_SYSTEM_NAME is "Generic".
##########################################################################
set(CMAKE_SYSTEM_INCLUDE_PATH "${CMAKE_FIND_ROOT_PATH}/include")
set(CMAKE_SYSTEM_LIBRARY_PATH "${CMAKE_FIND_ROOT_PATH}/lib")

##########################################################################
# status messages
##########################################################################
message(STATUS "Using toolchain file ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "Set CMAKE_FIND_ROOT_PATH to ${CMAKE_FIND_ROOT_PATH}")
message(STATUS "Set CMAKE_SYSTEM_INCLUDE_PATH to ${CMAKE_SYSTEM_INCLUDE_PATH}")
message(STATUS "Set CMAKE_SYSTEM_LIBRARY_PATH to ${CMAKE_SYSTEM_LIBRARY_PATH}")

##########################################################################
# set compiler options for build types
# Debug symbols are enabled to allow disassembly interleaved with C-code. 
##########################################################################
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_ASM_FLAGS_RELEASE "-O2 -g2 -gstabs")
    set(CMAKE_C_FLAGS_RELEASE "-O2 -g2 -gstabs")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g2 -gstabs")
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_ASM_FLAGS_DEBUG "-O2 -g2 -gstabs")
    set(CMAKE_C_FLAGS_DEBUG "-O2 -g2 -gstabs")
    set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g2 -gstabs")
elseif(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
    set(CMAKE_ASM_FLAGS_MINSIZEREL "-Os -g2 -gstabs")
    set(CMAKE_C_FLAGS_MINSIZEREL "-Os -g2 -gstabs")
    set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -g2 -gstabs")
elseif(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(CMAKE_ASM_FLAGS_RELWITHDEBINFO "-O2 -g2 -gstabs")
    set(CMAKE_C_FLAGS_RELWITHDEBINFO "-O2 -g2 -gstabs")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g2 -gstabs")
endif()


##################################################################################
# add AVR executable to project
# list all files included in the build
##################################################################################
add_avr_executable(HWPRobot
        lib/communication/communication.c
        lib/communication/communication.h
        lib/communication/packetTypes.h
        lib/io/adc/adc.c
        lib/io/adc/adc.h
        lib/io/adc/adc_isr.S
        lib/io/i2c/i2c.c
        lib/io/i2c/i2c.h
        lib/io/led/led.h
        lib/io/uart/uart.c
        lib/io/uart/uart.h
        lib/io/uart/uart_isr.S
        lib/motor/motor.c
        lib/motor/motor.h
        lib/pathFollower/pathFollower.c
        lib/pathFollower/pathFollower.h
        lib/tools/fifo/fifo.h
        lib/tools/labyrinth/labyrinth.c
        lib/tools/labyrinth/labyrinth.h
        lib/tools/remoteDataProcessing/remoteDataProcessing.c
        lib/tools/remoteDataProcessing/remoteDataProcessing.h
        lib/tools/timeTask/GPIOR0Defs.h
        lib/tools/timeTask/timeTask.c
        lib/tools/timeTask/timeTask.h
        lib/tools/timeTask/timeTask_isr.S
        lib/tools/powerSaver.h
        lib/tools/variablesAccess.h
        src/cfg/io/adc/adc_cfg.h
        src/cfg/io/uart/uart_cfg.h
        src/badISR.c
        src/main.c
        src/tasks/snake.h
        src/tasks/snake.c
        src/tasks/tasks.h
        src/tasks/tasks.c
        src/tasks/taskqueue.c
        src/tasks/taskqueue.h
        src/tasks/taskMethods.c
        src/tasks/taskMethods.h
        src/tasks/taskManagement.c
        src/tasks/taskManagement.h
        src/sensors/initSensors.c
        src/sensors/ISRCustom.c
        src/sensors/ISRCustom.h
        src/sensors/sensors.c
        src/sensors/sensors.h
        src/sensors/vision.c
        src/sensors/vision.h
        src/channels/channels.h
        src/channels/channels.c
        src/driving/driving.h
        src/driving/driving.c
        src/pose/pose.c
        src/pose/pose.h
        src/path/path.c
        src/path/path.h
        src/explorer/explorer.c
        src/explorer/explorer.h
        src/explorer/robot.c
        src/explorer/robot.h
        src/explorer/labyrinthState.c
        src/explorer/labyrinthState.h
        src/tests/test.c
        src/tests/test.h
        src/helper/mathHelper.c
        src/helper/mathHelper.h)

##########################################################################
# include search paths
##########################################################################
avr_target_include_directories(HWPRobot
	lib
	src
)

##########################################################################
# compiler definitions for all build types
##########################################################################
avr_target_compile_definitions(HWPRobot
	F_CPU=${MCU_SPEED}
)

##########################################################################
# compiler options for all build types
##########################################################################
avr_target_compile_options(HWPRobot
	-fpack-struct
	-fshort-enums
	-funsigned-char
	-funsigned-bitfields
	-ffunction-sections
	-fdata-sections
	-Wall
	-Wextra
	-Wpedantic
	-std=gnu99
)


##################################################################################
# link against libraries
##################################################################################
avr_target_link_libraries(HWPRobot
	-Wl,-lm
    -Wl,-lprintf_flt
)

##################################################################################
# linker options
##################################################################################
avr_target_link_options(HWPRobot
	-Wl,--gc-sections
	-mrelax
    -Wl,-u,vfprintf
)
