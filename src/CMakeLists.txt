file(GLOB atlaas_SRCS "*.cpp")
add_library( atlaas SHARED ${atlaas_SRCS} )
target_link_libraries( atlaas ${GDALWRAP_LIBRARIES} )
target_link_libraries( atlaas ${POINTMATCHER_LIBRARIES} )
install(TARGETS atlaas DESTINATION ${CMAKE_INSTALL_LIBDIR})
install_pkg_config_file(atlaas
    DESCRIPTION "Atlas at LAAS"
    CFLAGS
    LIBS -latlaas
    REQUIRES gdalwrap libpointmatcher
    VERSION ${PACKAGE_VERSION})
