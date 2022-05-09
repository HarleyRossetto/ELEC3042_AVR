cmake_minimum_required(VERSION 3.18)
project(ASSESSMENT_MINOR VERSION 0.1 
                DESCRIPTION "ELEC3042 Major Assessment."
                LANGUAGES C)

# Project Variables
set(BINARIES    ${CMAKE_CURRENT_SOURCE_DIR}/bin)

####################################################################################################
# Major Assessment
set(TARGET 
        "assessment_major")
set(SOURCES 
        "src/${TARGET}/main.c"
        "src/${TARGET}/iotypes.h"
        "src/${TARGET}/null.h"
        "src/${TARGET}/fsm.h" 
        "src/${TARGET}/fsm.c"
        "src/${TARGET}/bool.h"
        "src/${TARGET}/timers.h"
        "src/${TARGET}/button.h"
        "src/${TARGET}/button.c"
        #"src/${TARGET}/systemtimer.h"
        #"src/${TARGET}/systemtimer.c"
        "src/${TARGET}/timerTask.h"
        "src/${TARGET}/timerTask.c"
        "src/${TARGET}/led.h"
        "src/${TARGET}/led.c"
        "src/${TARGET}/flag.h"
        "src/${TARGET}/flag.c"
        "src/${TARGET}/types.h"
        "src/${TARGET}/drivers/i2c/display/22S1_ELEC3042_I2C_PCF8574.h"
        "src/${TARGET}/drivers/i2c/display/22S1_ELEC3042_I2C_PCF8574.c"
        "src/${TARGET}/drivers/spi/spi.h"
        "src/${TARGET}/drivers/spi/spi.c"
        "src/${TARGET}/drivers/mcp23s17/mcp23s17.h"
        "src/${TARGET}/drivers/mcp23s17/mcp23s17.c"
        "src/${TARGET}/intersection/light.h"
        "src/${TARGET}/intersection/light.c"
        "src/${TARGET}/intersection/trafficLight.h"
        "src/${TARGET}/intersection/trafficLight.c"
        )

####################################################################################################

set(TARGET_HEX ${TARGET}.hex)

####################################################################################################

# Utility Programs
# Programming
set(AVR_DUDE avrdude)
set(PROGRAMMER_TYPE arduino)
set(PROGRAMMER_ARGS )

if (WIN32)
    set(PROGRAMMER_PORT COM3)
elseif(UNIX AND NOT APPLE)
    set(PROGRAMMER_PORT /dev/ttyACM0)
else()
    set(PROGRAMMER_PORT set_programming_port)
endif()

####################################################################################################

# Output Files
set(TARGET_ELF ${TARGET}.elf)
set(TARGET_BIN ${TARGET}.bin)
set(TARGET_HEX ${TARGET}.hex)
set(TARGET_ASM ${TARGET}.asm)
set(TARGET_MAP ${TARGET}.map)
set(TARGET_SYM ${TARGET}.sym)

# Directories
set(TARGET_BIN_DIR ${BINARIES}/${TARGET}/)

set(TARGET_HEX_FILE ${TARGET_BIN_DIR}/hex/${TARGET_HEX})

####################################################################################################

# Chip Variables
set(MCU atmega328p)
set(MCU_OPT ATmega328)
set(CPU_FREQ 16000000UL)
set(BAUD 9600)

# Language Variables
set(CSTD gnu99)

####################################################################################################

message("Target: " ${TARGET} " Sources: " ${SOURCES} )

# Create the executable for each target, adding sources.
add_executable(${TARGET_HEX} ${SOURCES})

set_target_properties(${TARGET_HEX}
                    PROPERTIES
                        RUNTIME_OUTPUT_DIRECTORY ${TARGET_BIN_DIR}/hex
                        LIBRARY_OUTPUT_DIRECTORY ${TARGET_BIN_DIR}/lib
                        ARCHIVE_OUTPUT_DIRECTORY ${TARGET_BIN_DIR}/lib
) 

target_compile_definitions(${TARGET_HEX}
                            PUBLIC
                                F_CPU=${CPU_FREQ}
                                BAUD=${BAUD}
                                __${MCU_OPT}__
) 

target_include_directories(${TARGET_HEX}
                        PRIVATE
                            #${CMAKE_CURRENT_SOURCE_DIR}/src/ # Not sure that this is needed.
                            #${CMAKE_CURRENT_SOURCE_DIR}/include
                            ${CMAKE_CURRENT_SOURCE_DIR}/lib/include
)

target_compile_options(${TARGET_HEX}
                    PUBLIC
                        -mmcu=${MCU}            # MCU
                        -std=${CSTD}            # C Standard
                        -Os                     # Size

                        -Wall                   # Enable Warnings
                        -Wunused-function       # Warnings for unused functions.
                        
                        -ffunction-sections     # Place each function or data item into its own section in the output file
                        -fdata-sections         # if the target supports arbitrary sections.
                                                # Use these options where the linker can perform optimisations to improve locality
                                                # of reference in the instruction space.

                        -fshort-enums           # Allocate to an 'enum' type only as many bytes as it needs for the delcared range of possible values.
                                                # The 'enum' type will be equivalent to the smallest integer type which has enough room.

                        -fno-common             # Controls the placement of uninitialised global variables in C code.
                                                # Specifies that the compiler should place uninitialised global variables in the data section
                                                # of the object file, rather than generating them as common blocks.
                                                # Will give a multiple definition error if the symbol is declared more than once.

                        -funsigned-char         # Let the type 'char' be unsigned, like 'unsigned char'.

                        -funsigned-bitfields    # Controls whether a bit-field is signed of unsigned. In this case unsigned.
)

target_link_options(${TARGET_HEX}
                    PUBLIC
                        -mmcu=${MCU}
                        -Wl,--gc-sections       # Pass option as an option to the Linker
)

####################################################################################################

#
#   Target Custom Commands
#

# Generates approximiation of assembly code based on elf
add_custom_target(  disassemble-${TARGET}
                    DEPENDS ${TARGET_HEX})

add_custom_command( TARGET disassemble-${TARGET}
                    USES_TERMINAL
                    COMMAND ${CMAKE_OBJDUMP} -C -I -d -S ${TARGET_HEX_FILE} > ${TARGET_BIN_DIR}/disassembly/${TARGET_ASM})

# Flashes the MCU with the executable
add_custom_target(flash-${TARGET}
                  DEPENDS ${TARGET_HEX})
add_custom_command(TARGET flash-${TARGET}
                    USES_TERMINAL
                    COMMAND ${AVR_DUDE} -v -c ${PROGRAMMER_TYPE} -p ${MCU} ${PROGRAMMER_ARGS} -P ${PROGRAMMER_PORT} -b115200 -U flash:w:${b}${TARGET_HEX_FILE})