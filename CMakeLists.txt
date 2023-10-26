cmake_minimum_required(VERSION 2.8)
project( Emer_Handheld )
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package( OpenCV REQUIRED )
include_directories( include
		     ${OpenCV_INCLUDE_DIRS}
		     /opt/EVT/eSDK/include
 )
link_directories(/opt/EVT/eSDK/lib)
add_executable( Emer_Handheld Emer_Handheld.cpp )
target_link_libraries( Emer_Handheld
			${OpenCV_LIBS}			      
                       -lEmergentCamera
                       -lEmergentGenICam
                       -lEmergentGigEVision
                       -lboost_system 
                       -lboost_filesystem
)
