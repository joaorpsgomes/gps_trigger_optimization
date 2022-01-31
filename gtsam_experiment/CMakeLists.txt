cmake_minimum_required(VERSION 3.16)
project(graph_optimization_using_gps)




string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")






set(CMAKE_CXX_STANDART 11)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)
enable_testing()
set(EIGEN_INCLUDE_PATH PATH /usr/local/include/eigen3)
INCLUDE_DIRECTORIES ( ${EIGEN_INCLUDE_PATH} )

find_package(RTABMap 0.20.9 REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Ceres REQUIRED)
find_package(g2o REQUIRED)

IF(RTABMAP_GUI)
FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation visualization)
ELSE()
FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation )
ENDIF()
add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set


option(RTABMAP_SYNC_MULTI_RGBD "Build with multi RGBD camera synchronization support"  OFF)
option(RTABMAP_SYNC_USER_DATA "Build with input user data support"  OFF)
MESSAGE(STATUS "RTABMAP_SYNC_MULTI_RGBD = ${RTABMAP_SYNC_MULTI_RGBD}")
MESSAGE(STATUS "RTABMAP_SYNC_USER_DATA  = ${RTABMAP_SYNC_USER_DATA}")
IF(RTABMAP_SYNC_MULTI_RGBD)
add_definitions("-DRTABMAP_SYNC_MULTI_RGBD")
ENDIF(RTABMAP_SYNC_MULTI_RGBD)
IF(RTABMAP_SYNC_USER_DATA)
add_definitions("-DRTABMAP_SYNC_USER_DATA")
ENDIF(RTABMAP_SYNC_USER_DATA)



SET(PACKAGE RTABMAP)
IF (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldGreen}Found ${PACKAGE}${ColourReset}")
ELSE (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldRed}${PACKAGE} Not Found${ColourReset}")
ENDIF (${${PACKAGE}_FOUND})

SET(PACKAGE PCL)
IF (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldGreen}Found ${PACKAGE}${ColourReset}")
ELSE (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldRed}${PACKAGE} Not Found${ColourReset}")
ENDIF (${${PACKAGE}_FOUND})

SET(PACKAGE OpenCV)
IF (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldGreen}Found ${PACKAGE}${ColourReset}")
ELSE (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldRed}${PACKAGE} Not Found${ColourReset}")
ENDIF (${${PACKAGE}_FOUND})

SET(PACKAGE GTSAM)
IF (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldGreen}Found ${PACKAGE}${ColourReset}")
ELSE (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldRed}${PACKAGE} Not Found${ColourReset}")
ENDIF (${${PACKAGE}_FOUND})

SET(PACKAGE Ceres)
IF (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldGreen}Found ${PACKAGE}${ColourReset}")
ELSE (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldRed}${PACKAGE} Not Found${ColourReset}")
ENDIF (${${PACKAGE}_FOUND})

SET(PACKAGE g2o)
IF (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldGreen}Found ${PACKAGE}${ColourReset}")
ELSE (${${PACKAGE}_FOUND})
	MESSAGE(STATUS "${BoldRed}${PACKAGE} Not Found${ColourReset}")
ENDIF (${${PACKAGE}_FOUND})


include_directories( 
  ${RTABMap_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIRS}
)
file(GLOB DisplayImage_SRCS
    "*.h"
    "*.cpp")

SET(Libraries
   ${RTABMap_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Ceres_LIBRARIES}
   ${GTSAM_LIBRARIES}
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

add_executable(optimizer 
               src/main.cpp 
               ${OpenCV_INCLUDE_DIRS}
               ${PCL_INCLUDE_DIRS}
               ${RTABMap_INCLUDE_DIRS}
               ${Ceres_INCLUDE_DIRS}
               ${GTSAM_INCLUDE_DIRS}
               )



add_library(gps_handler
            src/gps_handler.cpp
            include/gps_handler.h)
            
            
               
target_link_libraries( optimizer ${Libraries} gps_handler )

