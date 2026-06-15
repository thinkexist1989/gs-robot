cmake_minimum_required(VERSION 3.16)

project(SpaceBuilding LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ============================================================
# 第三方依赖
# ============================================================

# SOEM 1.4.0（从 third_party 源码构建）
# SOEM 自带 -Werror，GCC 11 上的 strncpy 长度推断警告会导致编译失败，
# 通过 target_compile_options 追加 -Wno-error 覆盖。
# SOEM 使用旧式 include_directories()，不会自动传播，需手动设置 INTERFACE。
set(SOEM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/SOEM-1.4.0)
set(CMAKE_C_FLAGS_SAVE "${CMAKE_C_FLAGS}")
add_subdirectory(${SOEM_ROOT} third_party/SOEM-1.4.0 EXCLUDE_FROM_ALL)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS_SAVE}")
target_compile_options(soem PRIVATE -Wno-error)
set_target_properties(soem PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES
        "${SOEM_ROOT}/soem;${SOEM_ROOT}/osal;${SOEM_ROOT}/osal/linux;${SOEM_ROOT}/oshw/linux"
)

# ruckig（从 third_party 源码构建）
set(BUILD_EXAMPLES     OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS        OFF CACHE BOOL "" FORCE)
set(BUILD_BENCHMARK    OFF CACHE BOOL "" FORCE)
set(BUILD_CLOUD_CLIENT OFF CACHE BOOL "" FORCE)
set(BUILD_PYTHON_MODULE OFF CACHE BOOL "" FORCE)
add_subdirectory(third_party/ruckig-0.17.3 EXCLUDE_FROM_ALL)

# orocos-kdl（从 third_party 源码构建）
add_subdirectory(
    third_party/orocos_kinematics_dynamics-1.5.3/orocos_kdl
    EXCLUDE_FROM_ALL
)

# ============================================================
# Qt
# ============================================================
find_package(Qt6 COMPONENTS Core Gui Widgets SerialPort Network QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort Network REQUIRED)
    set(QT_VERSION_MAJOR 5)
else()
    set(QT_VERSION_MAJOR 6)
endif()

# ============================================================
# 主目标
# ============================================================
set(SOURCES
    common.cpp
    ethercatthread.cpp
    main.cpp
    mainwindow.cpp
    robotdescription.cpp
    socketthread.cpp
    straightlinetrajectorygenerator.cpp
    vision.cpp
)

set(HEADERS
    common.h
    ethercatthread.h
    mainwindow.h
    robotdescription.h
    socketthread.h
    straightlinetrajectorygenerator.h
    vision.h
)

set(FORMS
    mainwindow.ui
)

set(RESOURCES
    resources.qrc
)

add_executable(${PROJECT_NAME}
    ${SOURCES}
    ${HEADERS}
    ${FORMS}
    ${RESOURCES}
)

if(QT_VERSION_MAJOR EQUAL 6)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::SerialPort
        Qt6::Network
    )
else()
    target_link_libraries(${PROJECT_NAME} PRIVATE
        Qt5::Core
        Qt5::Gui
        Qt5::Widgets
        Qt5::SerialPort
        Qt5::Network
    )
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    /usr/include/eigen3
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    soem
    orocos-kdl
    ruckig
)

if(UNIX AND NOT APPLE)
    install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION /opt/${PROJECT_NAME}/bin
    )
endif()
