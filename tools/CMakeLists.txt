add_executable(atlaas_dem dem.cpp)
target_link_libraries( atlaas_dem atlaas )
target_link_libraries( atlaas_dem ${GDALWRAP_LIBRARIES} )
install(TARGETS atlaas_dem DESTINATION bin)

add_executable(atlaas_process process.cpp)
target_link_libraries( atlaas_process atlaas )
install(TARGETS atlaas_process DESTINATION bin)
