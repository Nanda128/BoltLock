FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

# Add U8g2 component from PlatformIO library
set(U8G2_DIR "${CMAKE_SOURCE_DIR}/.pio/libdeps/firebeetle32/U8g2")
if(EXISTS ${U8G2_DIR})
    file(GLOB U8G2_SOURCES 
        "${U8G2_DIR}/src/clib/*.c"
    )
    
    idf_component_register(
        SRCS ${app_sources} ${U8G2_SOURCES}
        INCLUDE_DIRS 
            "${CMAKE_SOURCE_DIR}/include"
            "${U8G2_DIR}/src"
            "${U8G2_DIR}/src/clib"
    )
else()
    idf_component_register(SRCS ${app_sources})
endif()
