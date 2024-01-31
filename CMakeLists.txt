# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
cmake_minimum_required (VERSION 3.12)

project (triplependulum)

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

## find Eigen
IF(WIN32)
    set(TARGET_EIGEN_PATH "" CACHE PATH "Eigen3 install path")
    if(EXISTS ${TARGET_EIGEN_PATH})
        message(STATUS "Directory to search Eigen3 at ${TARGET_EIGEN_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_EIGEN_PATH})
    else()
        message(WARNING "File/Directory at variable TARGET_EIGEN_PATH not exists!")
    endif()
endif(WIN32)
find_package(Eigen3 REQUIRED)

## find osqp
IF(WIN32)
    set(TARGET_OSQP_PATH "" CACHE PATH "Osqp OSQP install path")
    if(EXISTS ${TARGET_OSQP_PATH})
        message(STATUS "Directory to search Osqp at ${TARGET_OSQP_PATH}")
        list(APPEND CMAKE_PREFIX_PATH ${TARGET_OSQP_PATH})
    else()
        message(WARNING "File/Directory at variable TARGET_OSQP_PATH not exists!")
    endif()
endif(WIN32)
find_package(osqp REQUIRED)

# 包含头文件
include_directories(${aris_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(${osqp_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

IF(WIN32)
    set(BUILD_OBJECT_NAME bal_ctrl ${BUILD_OBJECT_NAME})
endif(WIN32)
if(UNIX)
endif(UNIX)

file(GLOB_RECURSE ALL_LIBRARY_SRCS "src/*.cpp")
file(GLOB_RECURSE ALL_LIBRARY_HDRS "include/*.hpp" "include/*.h")

add_library (${BUILD_OBJECT_NAME} SHARED)
target_compile_definitions(${BUILD_OBJECT_NAME} PRIVATE EXPORT)
target_sources(${BUILD_OBJECT_NAME} 
    PRIVATE
        "src/controller/control.cpp"
        "src/controller/model.cpp"
        "src/plan/plan.cpp"
        "src/solver/math.cpp"
        "src/solver/math.hpp"
        "src/solver/qpsolver.cpp"
        "src/solver/qpsolver.hpp"
        "src/zmq/zmqmsg.cpp"
        "src/zmq/statcal_util.cpp"
        "src/zmq/statcal_util.hpp"
    PUBLIC
        ${ALL_LIBRARY_HDRS}
    )

# 动态库包含的地址
target_include_directories(${BUILD_OBJECT_NAME}
    PUBLIC 
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${PROJECT_SOURCE_DIR}/lib"
        "${PROJECT_SOURCE_DIR}/cppzmq"
    PRIVATE
        "${PROJECT_SOURCE_DIR}/src"
        "${PROJECT_SOURCE_DIR}")

## 将需要的动态库链接到 triplependulum 中
target_link_libraries(${BUILD_OBJECT_NAME} ${aris_LIBRARIES})
if (TARGET Eigen3::Eigen)
    # Use the imported target
    target_link_libraries(${BUILD_OBJECT_NAME} Eigen3::Eigen)
endif (TARGET Eigen3::Eigen)
if (TARGET osqp::osqp)
    # Use the imported target
    target_link_libraries(${BUILD_OBJECT_NAME} osqp::osqp)
endif (TARGET osqp::osqp)

IF(WIN32)
    target_link_libraries(${BUILD_OBJECT_NAME} ${PROJECT_SOURCE_DIR}/lib/libzmq.lib)
    target_link_libraries(${BUILD_OBJECT_NAME} ${PROJECT_SOURCE_DIR}/lib/libzmq-v142-mt-4_3_4.lib)
endif(WIN32)

install(TARGETS ${BUILD_OBJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

IF(WIN32)
    ## copy .dll 到 out 文件夹下
    add_custom_command(TARGET ${BUILD_OBJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
            "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      # <--this is in-file
            $<TARGET_FILE_DIR:${BUILD_OBJECT_NAME}>)                 # <--this is out-file path

    add_custom_command(TARGET ${BUILD_OBJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
            "${PROJECT_SOURCE_DIR}/lib/libzmq-v142-mt-4_3_4.dll"    # <--this is in-file
            "${PROJECT_SOURCE_DIR}/lib/libsodium.dll" 
            $<TARGET_FILE_DIR:${BUILD_OBJECT_NAME}>)                 # <--this is out-file path

    add_custom_command(TARGET ${BUILD_OBJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
            "${osqp_DIR}/../../../bin/osqp.dll"        # <--this is in-file   #Different versions of osqp have different names and this is an error prone area
            $<TARGET_FILE_DIR:${BUILD_OBJECT_NAME}>)                 # <--this is out-file path
endif(WIN32)

option(BUILD_DEMOS "build demos" OFF)
if(BUILD_DEMOS)
    set(DEMO_NAME triple_demo)
    add_executable(${DEMO_NAME} "demo/main.cpp")
    target_include_directories(${DEMO_NAME} 
        PRIVATE 
            ${aris_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_directories(${DEMO_NAME}
        PRIVATE 
            ${PROJECT_BINARY_DIR})
    target_link_libraries(${DEMO_NAME}
        ${aris_LIBRARIES}
        ${BUILD_OBJECT_NAME})
endif(BUILD_DEMOS)