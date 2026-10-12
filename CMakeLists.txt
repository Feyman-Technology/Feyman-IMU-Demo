cmake_minimum_required(VERSION 2.8)

project(FmiIMUDemo)

file( GLOB_RECURSE CORE_SRC_FILES "./Device/Src/*.cpp")
file( GLOB_RECURSE CORE_INC_FILES "./Device/Inc/*.h")

#header files
set( CORE_INC_DIRS "" )
foreach( _headerFile ${CORE_INC_FILES} )
    get_filename_component( _dir ${_headerFile} PATH )
    list( APPEND CORE_INC_DIRS ${_dir} )
endforeach()
list( REMOVE_DUPLICATES CORE_INC_DIRS )
include_directories( ${CORE_INC_DIRS} )

#sources in root directory
aux_source_directory(. ROOT_DIR_SRCS)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O2 -Wall")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

add_executable(${PROJECT_NAME} ${ROOT_DIR_SRCS} ${CORE_SRC_FILES} ${CORE_INC_FILES})

