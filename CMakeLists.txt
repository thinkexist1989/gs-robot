cmake_minimum_required(VERSION 3.16)

project(SpaceBuilding LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 COMPONENTS Core Gui Widgets SerialPort Network QUIET)
if(NOT Qt6_FOUND)
    find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort Network REQUIRED)
    set(QT_VERSION_MAJOR 5)
else()
    set(QT_VERSION_MAJOR 6)
endif()

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
    /usr/local/include/soem
    /usr/include/kdl
    /opt/rocos/app/include
)

target_link_directories(${PROJECT_NAME} PRIVATE
    /usr/local/lib
    /opt/rocos/app/lib
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
