cmake_minimum_required(VERSION 2.8)

set(PACKAGE_NAME atlaas)
set(PACKAGE_VERSION "0.1.4")

option(BUILD_TESTS  "Build tests" ON)
option(BUILD_TOOLS  "Build tools" OFF)
option(BUILD_DOC    "Build documentation" OFF)

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# C++11 GDAL wrapper
find_package(GdalWrap REQUIRED)

include_directories(include)
include_directories(${GDALWRAP_INCLUDE_DIRS})

# Load the 'install_pkg_config_file' macro
include(InstallPkgConfigFile)



# find octomap (for gdal wrapper)
pkg_check_modules(PC_OCTOMAP REQUIRED octomap)
set(OCTOMAP_DEFINITIONS ${PC_OCTOMAP_CFLAGS_OTHER})
find_path(OCTOMAP_INCLUDE_DIR octomap/octomap.h
    HINTS ${PC_OCTOMAP_INCLUDEDIR} ${PC_OCTOMAP_INCLUDE_DIRS}
    PATH_SUFFIXES octomap)
find_library(OCTOMAP_LIBRARY NAME octomap
    HINTS ${PC_OCTOMAP_LIBDIR} ${PC_OCTOMAP_LIBRARY_DIRS} )
set(OCTOMAP_INCLUDE_DIRS ${OCTOMAP_INCLUDE_DIR})
set(OCTOMAP_LIBRARIES ${OCTOMAP_LIBRARY})
##
include_directories(${OCTOMAP_INCLUDE_DIRS})



# RPath management http://www.cmake.org/Wiki/CMake_RPATH_handling
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif("${isSystemDir}" STREQUAL "-1")


# Library and binary
add_subdirectory(src)

if (BUILD_TOOLS)
  add_subdirectory(tools)
endif(BUILD_TOOLS)

if (BUILD_TESTS)
  include( CTest )
  add_subdirectory(test)
endif(BUILD_TESTS)

if (BUILD_DOC)
  find_package(Doxygen)
  if(DOXYGEN_FOUND)
    configure_file( doxyfile.dox.in ${CMAKE_CURRENT_BINARY_DIR}/doxyfile.dox @ONLY)
    add_custom_target( doc ALL
      ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile.dox
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
      COMMENT "Generating documentation (Doxygen)" VERBATIM )
  endif(DOXYGEN_FOUND)
endif(BUILD_DOC)

# Install headers
file(GLOB atlaas_HDRS "include/atlaas/*.hpp")
install(FILES ${atlaas_HDRS} DESTINATION include/atlaas)

#########################################################################
### Uninstall ###
#########################################################################
configure_file("${CMAKE_HOME_DIRECTORY}/CMakeModules/cmake_uninstall.cmake.in" "cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "cmake_uninstall.cmake")
