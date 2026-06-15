cmake_minimum_required(VERSION 3.16)
project(crdm_scram_audit LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/O2 /Ob2 /Oi /Ot /Oy /GL /GS- /arch:AVX2 /fp:fast)
    add_link_options(/LTCG)
else()
    add_compile_options(-O3 -march=native -ffast-math -funroll-loops -flto)
    add_link_options(-flto)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

add_executable(crdm_scram_audit ${SOURCES})

if(WIN32)
    target_link_libraries(crdm_scram_audit PRIVATE)
else()
    target_link_libraries(crdm_scram_audit PRIVATE pthread)
endif()
