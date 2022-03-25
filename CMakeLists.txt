cmake_minimum_required(VERSION 3.18)
project(AVR_328 VERSION 0.1 
                DESCRIPTION "ELEC3042 AVR Build System Template."
                LANGUAGES C)

include(src/assessment_minor/CMakeLists.txt)
include(src/main/CMakeLists.txt)
include(src/week3/w3p1/CMakeLists.txt)

include(src/week4/p1/CMakeLists.txt)
include(src/week4/p2/CMakeLists.txt)
include(src/week4/p3/CMakeLists.txt)

include(src/week5/i2c/CMakeLists.txt)
include(src/week5/p1/CMakeLists.txt)

include(src/spi/CMakeLists.txt)