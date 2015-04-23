add_executable(atlaas_dem dem.cpp)
target_link_libraries( atlaas_dem atlaas )
target_link_libraries( atlaas_dem ${GDALWRAP_LIBRARIES} )
install(TARGETS atlaas_dem DESTINATION bin)

add_executable(atlaas_process process.cpp)
target_link_libraries( atlaas_process atlaas )
install(TARGETS atlaas_process DESTINATION bin)

add_executable(pcd2atlaas pcd2atlaas.cpp)
target_link_libraries( pcd2atlaas atlaas )
install(TARGETS pcd2atlaas DESTINATION bin)

add_executable(atlaas2pcd atlaas2pcd.cpp)
target_link_libraries( atlaas2pcd atlaas )
install(TARGETS atlaas2pcd DESTINATION bin)

add_executable(global_region global_region.cpp)
target_link_libraries( global_region atlaas )
install(TARGETS global_region DESTINATION bin)

add_executable(global_heightmap global_heightmap.cpp)
target_link_libraries( global_heightmap atlaas )
install(TARGETS global_heightmap DESTINATION bin)
