# find libpointmatcher (ICP)
find_path(POINTMATCHER_INCLUDE_DIR pointmatcher/PointMatcher.h
    HINTS /usr/local/include
    PATH_SUFFIXES pointmatcher
)
find_library(POINTMATCHER_LIBRARY NAMES libpointmatcher pointmatcher
    HINTS /usr/local/lib
)

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(POINTMATCHER DEFAULT_MS
    POINTMATCHER_LIBRARY POINTMATCHER_INCLUDE_DIR
)

find_path(EIGEN3_INCLUDE_DIR Eigen/Eigen
    HINTS /usr/include
    PATH_SUFFIXES eigen3
)
find_package(Boost COMPONENTS thread filesystem system program_options date_time REQUIRED)
# nabo
find_path(NABO_INCLUDE_DIR nabo/nabo.h
	/usr/local/include
)
find_library(NABO_LIBRARY NAMES libnabo.a nabo PATHS
	/usr/local/lib
)
find_package(PkgConfig)

pkg_check_modules(PC_yaml-cpp REQUIRED yaml-cpp)
set(yaml-cpp_DEFINITIONS ${PC_yaml-cpp_CFLAGS_OTHER})
find_path(yaml-cpp_INCLUDE_DIR yaml-cpp/yaml.h
    HINTS ${PC_yaml-cpp_INCLUDEDIR} ${PC_yaml-cpp_INCLUDE_DIRS}
    PATH_SUFFIXES yaml-cpp)
find_library(yaml-cpp_LIBRARY NAME yaml-cpp
    HINTS ${PC_yaml-cpp_LIBDIR} ${PC_yaml-cpp_LIBRARY_DIRS} )
##

set(POINTMATCHER_INCLUDE_DIRS ${POINTMATCHER_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${NABO_INCLUDE_DIR} ${yaml-cpp_INCLUDE_DIR})
set(POINTMATCHER_LIBRARIES ${POINTMATCHER_LIBRARY} ${Boost_LIBRARIES} ${NABO_LIBRARY} ${yaml-cpp_LIBRARY})
