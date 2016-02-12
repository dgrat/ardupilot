# Build script for ArduPilot.
#
# Author: Daniel Frenzel
#

cmake_minimum_required(VERSION 3.0.2)
project(ArduCopter)


AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR} SUB_SOURCES) 
SET( ArduCopter
  ${SUB_SOURCES}
)

message( STATUS "Include ArduPilot library directories for ArduCopter" )
foreach( DIR ${AP_List} ${AP_List_Linux} ${AP_Headers} )
	include_directories( "../libraries/${DIR}" )
endforeach()

# ***************************************
# Build the firmware
# ***************************************
ADD_EXECUTABLE ( ArduCopter ${ArduCopter} ${PROJECT_FILE} )

# Native or Linux build
target_link_libraries ( ArduCopter -Wl,--start-group ${AP_List} ${AP_List_Linux} -Wl,--end-group ) 