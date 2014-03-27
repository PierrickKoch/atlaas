cmake_minimum_required(VERSION 2.8)

set(PACKAGE_NAME atlaas)
set(PACKAGE_VERSION "0.1.3")

option(BUILD_TOOLS  "Build tools" OFF)
option(BUILD_DOC    "Build documation" OFF)

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# C++11 GDAL wrapper
find_package(GdalWrap REQUIRED)

include_directories(include)
include_directories(${GDALWRAP_INCLUDE_DIRS})

# Filesystem Hierarchy Standard
include(GNUInstallDirs)

# Load the 'install_pkg_config_file' macro
include(InstallPkgConfigFile)

# RPath management http://www.cmake.org/Wiki/CMake_RPATH_handling
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif("${isSystemDir}" STREQUAL "-1")


# Library and binary
add_subdirectory(src)

if (BUILD_TOOLS)
    add_subdirectory(tools)
endif(BUILD_TOOLS)

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
install(FILES ${atlaas_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/atlaas)

#########################################################################
### Uninstall ###
#########################################################################
configure_file("${CMAKE_HOME_DIRECTORY}/CMakeModules/cmake_uninstall.cmake.in" "cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "cmake_uninstall.cmake")
