cmake_minimum_required(VERSION 3.10.2) 
set (CMAKE_CXX_STANDARD 17)

project(Main)

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")

find_package( OpenCV REQUIRED  )    
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_library(ESENetCam_LIBRARY libESENetCam.so HINTS /usr/lib/)

add_executable (Main main.cpp cameradatamanager.cpp cameradatamanager.hpp analysis.cpp analysis.hpp)      
target_link_libraries(Main ${ESENetCam_LIBRARY} ${OpenCV_LIBS}) 
