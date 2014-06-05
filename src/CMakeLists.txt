file(GLOB atlaas_SRCS "*.cpp")
add_library( atlaas SHARED ${atlaas_SRCS} )
target_link_libraries( atlaas ${GDALWRAP_LIBRARIES} )
target_link_libraries( atlaas ${OCTOMAP_LIBRARIES} )
install(TARGETS atlaas DESTINATION lib)
install_pkg_config_file(atlaas
    DESCRIPTION "Atlas at LAAS"
    CFLAGS
    LIBS -latlaas
    REQUIRES gdalwrap
    VERSION ${PACKAGE_VERSION})
