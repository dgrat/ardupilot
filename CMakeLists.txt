# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 3.0.2)

###########################################################################
# Add include path' for cmake modules
###########################################################################
message( STATUS "Include cmake modules" )
list(APPEND CMAKE_MODULE_PATH 
  "${CMAKE_SOURCE_DIR}/cmake/modules" 
  "${CMAKE_SOURCE_DIR}/cmake/config" 
  "${CMAKE_SOURCE_DIR}/cmake/toolchains" 
)

###########################################################################
# Generate build flags and config files
# .. if required ..
###########################################################################
include(CMakeParseArguments)
include(APMOption)

message( STATUS "Include options file" )
unset(APM_BUILD_FLAGS_LIST CACHE) # Clear old CACHE values ..
include(options.cmake)            # Read current build options

message( STATUS "Generate definitions (build flags) from cmake option file (or console parameters)" )
apm_option_genflags(BUILD_FLAGS APM_BUILD_FLAGS)
#apm_option_genhfile(FILE "config.h")

###########################################################################
# Define here the toolchain to use
# Currently this just influences which libraries to link together
# E.g. Linux will require AP_HAL_Linux, which is not used for other
# build targets
# Later I may enforce application of toolchain files :D
###########################################################################
message( STATUS "Analyze the definitions for hardware/platform settings" )
apm_find_flag_parameter("${APM_BUILD_FLAGS}" "CONFIG_HAL_BOARD" BOARD_TYPE )

###########################################################################
# Add some ArduPilot related build flags (board type, ..)
###########################################################################
# Delete disbaled build flags
apm_delete_disabled_flags("${APM_BUILD_FLAGS}" ACTIVE_BUILD_FLAGS)
unset(APM_BUILD_FLAGS CACHE) 
set(APM_BUILD_FLAGS ${ACTIVE_BUILD_FLAGS}) 

# Print active build flags
message( STATUS "APM_BUILD_FLAGS are:")
foreach(FLAG ${APM_BUILD_FLAGS})
  message( STATUS "  " ${FLAG} )
endforeach()

# Add the definitions globally
add_definitions( ${APM_BUILD_FLAGS} )

###########################################################################
# Project
###########################################################################
project(ArduPilot)

###########################################################################
# Project Module Preparation
###########################################################################
include( git_submodule )
include( ExternalProject )

# Build MavLink headers and install them into the library
ExternalProject_Add( MAVLink 
                     SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/modules/mavlink 
                     BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/MAVLink
                     CMAKE_ARGS 
                       -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
                       -DCMAKE_INSTALL_PREFIX=${CMAKE_CURRENT_SOURCE_DIR}/libraries/GCS_MAVLink/
)

###########################################################################
# Project Configuration
###########################################################################
include( ap-libraries )
include( ap-flags )

###########################################################################
# Check for the presence of some modules
###########################################################################
find_package( i2c-tools )

###########################################################################
# Library module list
###########################################################################
message( STATUS "Include ArduPilot library top directory: ${CMAKE_CURRENT_SOURCE_DIR}/libraries" )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/libraries )

###########################################################################
# Compiler and linker settings
###########################################################################
set(CMAKE_C_FLAGS           "${CMAKE_C_FLAGS}   ${REQ_FLAGS} ${OPT_FLAGS} ${WRN_FLAGS}")
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} ${REQ_FLAGS} ${OPT_FLAGS} ${WRN_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -Wl,--relax")

###########################################################################
# Build the library
###########################################################################
message( STATUS "Walk through the sub directories of the library" )
add_subdirectory( libraries )

###########################################################################
# Build the vehicle code
###########################################################################
# Which sketch was defined to be build?
apm_find_flag_parameter("${APM_BUILD_FLAGS}" "SKETCHNAME" SKETCHN )
if( ${SKETCHN} STREQUAL "" )
  message( FATAL_ERROR "NO build target defined" )
else()
  # Finally, build!
  include_directories ( ${SKETCHN} )
  add_subdirectory( ${SKETCHN} )
  message( STATUS "Build target: ${SKETCHN}" )
endif()

