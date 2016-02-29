include_directories(.)

macro(add_atlaas_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} atlaas)
    add_test(${name} ${name})
endmacro()

add_atlaas_test( merge_test )
add_atlaas_test( pcd_test )
add_atlaas_test( io_test )
add_atlaas_test( io_c_test )
