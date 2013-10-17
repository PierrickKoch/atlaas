cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(atlaas)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

####
# external stuff
#
find_package(PkgConfig)

# find atlaas
pkg_check_modules(PC_ATLAAS REQUIRED atlaas)
set(ATLAAS_DEFINITIONS ${PC_ATLAAS_CFLAGS_OTHER})
find_path(ATLAAS_INCLUDE_DIR atlaas/atlaas.hpp
    HINTS ${PC_ATLAAS_INCLUDEDIR} ${PC_ATLAAS_INCLUDE_DIRS}
    PATH_SUFFIXES atlaas)
find_library(ATLAAS_LIBRARY NAME atlaas
    HINTS ${PC_ATLAAS_LIBDIR} ${PC_ATLAAS_LIBRARY_DIRS} )
set(ATLAAS_INCLUDE_DIRS ${ATLAAS_INCLUDE_DIR})
set(ATLAAS_LIBRARIES ${ATLAAS_LIBRARY})

# find gdalwrap (for gdal wrapper)
pkg_check_modules(PC_GDALWRAP REQUIRED gdalwrap)
set(GDALWRAP_DEFINITIONS ${PC_GDALWRAP_CFLAGS_OTHER})
find_path(GDALWRAP_INCLUDE_DIR gdalwrap/gdal.hpp
    HINTS ${PC_GDALWRAP_INCLUDEDIR} ${PC_GDALWRAP_INCLUDE_DIRS}
    PATH_SUFFIXES gdalwrap)
find_library(GDALWRAP_LIBRARY NAME gdalwrap
    HINTS ${PC_GDALWRAP_LIBDIR} ${PC_GDALWRAP_LIBRARY_DIRS} )
set(GDALWRAP_INCLUDE_DIRS ${GDALWRAP_INCLUDE_DIR})
set(GDALWRAP_LIBRARIES ${GDALWRAP_LIBRARY})

include_directories(${ATLAAS_INCLUDE_DIRS})
include_directories(${GDALWRAP_INCLUDE_DIRS})

####

rosbuild_add_executable(atlaas src/atlaas.cpp)
target_link_libraries( atlaas ${ATLAAS_LIBRARIES} )
target_link_libraries( atlaas ${GDALWRAP_LIBRARIES} )
target_link_libraries( atlaas ${PCL_LIBRARIES} )
