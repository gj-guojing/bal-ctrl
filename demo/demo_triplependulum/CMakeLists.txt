# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
cmake_minimum_required (VERSION 3.12)

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

add_executable(${EXE_NAME} "main.cpp" )
# add_executable(${EXE_NAME} "test_dynamics.cpp" )

    
target_include_directories(${EXE_NAME} 
    PUBLIC
        ${aris_INCLUDE_DIRS}
        ${HEAD_PATH})

target_link_libraries(${EXE_NAME}
        ${aris_LIBRARIES}
        ${LIB_NAME})

IF(WIN32)
add_custom_command(TARGET ${EXE_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        "${LIB_PATH}/src/${LIB_NAME}.dll"
        $<TARGET_FILE_DIR:${EXE_NAME}>)
            ## copy .dll 到 out 文件夹下
add_custom_command(TARGET ${EXE_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
            "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      # <--this is in-file
            $<TARGET_FILE_DIR:${EXE_NAME}>) 
add_custom_command(TARGET ${EXE_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
            "${ZMQ_LIB}/lib/libzmq-v142-mt-4_3_4.dll"    # <--this is in-file
            "${ZMQ_LIB}/lib/libsodium.dll" 
            $<TARGET_FILE_DIR:${EXE_NAME}>)                 # <--this is out-file path
add_custom_command(TARGET ${EXE_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy                    # which executes "cmake - E copy_if_different..."
            "${osqp_DIR}/../../../bin/osqp.dll"             # <--this is in-file   #Different versions of osqp have different names and this is an error prone area
            $<TARGET_FILE_DIR:${EXE_NAME}>)                 # <--this is out-file path
endif(WIN32)
