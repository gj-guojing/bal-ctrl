# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
cmake_minimum_required (VERSION 3.12)

project (utpr_demo)

## aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

## find aris
if(UNIX)
	add_definitions(-DUNIX)
    find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
    find_package(aris REQUIRED PATHS C:/aris/aris-2.3.9.230818)
endif(WIN32)

add_executable(${EXE_NAME} "main.cpp")
    
target_include_directories(${EXE_NAME} 
    PUBLIC
        ${aris_INCLUDE_DIRS}
        ${HEAD_PATH})

target_link_libraries(${EXE_NAME}
        ${aris_LIBRARIES}
        ${LIB_NAME})

add_custom_command(TARGET ${EXE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "${LIB_PATH}/src/src.dll"
        $<TARGET_FILE_DIR:${EXE_NAME}>)