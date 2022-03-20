cmake_minimum_required(VERSION 3.18)
project(W4P1 VERSION 0.1 
                DESCRIPTION "ELEC3042 Week 4 Part 1."
                LANGUAGES C)

# Project Variables
set(BINARIES    ${CMAKE_CURRENT_SOURCE_DIR}/bin)

####################################################################################################
# Minor Assessment
set(TARGET
        "w4p1")
set(SOURCES 
        "src/week4/p1/main.c")


        
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
set(${TARGET}_ELF ${TARGET}.elf)
set(${TARGET}_BIN ${TARGET}.bin)
set(${TARGET}_HEX ${TARGET}.hex)
set(${TARGET}_ASM ${TARGET}.asm)
set(${TARGET}_MAP ${TARGET}.map)
set(${TARGET}_SYM ${TARGET}.sym)

# Directories
set(${TARGET}_HEX_FILE ${BINARIES}/${TARGET}/hex/${TARGET_HEX})

####################################################################################################

# Chip Variables
set(MCU atmega328p)
set(MCU_OPT ATmega328)
set(CPU_FREQ 16000000UL)
set(BAUD 9600)

# Language Variables
set(CSTD gnu99)

####################################################################################################

message("TARGET: " ${TARGET} " SOURCES: " ${SOURCES} )

# Create the executable for each target, adding sources.
add_executable(${TARGET_HEX} ${SOURCES})

set_target_properties(${TARGET_HEX}
                    PROPERTIES
                        RUNTIME_OUTPUT_DIRECTORY ${BINARIES}/${TARGET}/hex
                        LIBRARY_OUTPUT_DIRECTORY ${BINARIES}/${TARGET}/lib
                        ARCHIVE_OUTPUT_DIRECTORY ${BINARIES}/${TARGET}/lib
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
                        -Os                     # Optimise

                        -Wall                   # Enable Warnings
                        
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
                    COMMAND ${CMAKE_OBJDUMP} -C -I -d -S ${${TARGET}_HEX_FILE} > ${BINARIES}/${TARGET}/disassembly/${${TARGET}_ASM})

# Flashes the MCU with the executable
add_custom_target(flash-${TARGET}
                  DEPENDS ${TARGET_HEX})
add_custom_command(TARGET flash-${TARGET}
                    USES_TERMINAL
                    COMMAND ${AVR_DUDE} -v -c ${PROGRAMMER_TYPE} -p ${MCU} ${PROGRAMMER_ARGS} -P ${PROGRAMMER_PORT} -b115200 -U flash:w:${${TARGET}_HEX_FILE})  