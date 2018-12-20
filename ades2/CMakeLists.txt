cmake_minimum_required(VERSION 3.10)
project(ades2)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost 1.57.0 COMPONENTS system filesystem REQUIRED)

#MySQL Connector
set(MYSQLCONNECTORCPP_ROOT_DIR /home/peter/projects/source/mysql-connector-cpp-install)

find_path(MYSQLCONNECTORCPP_INCLUDE_DIR
        NAMES mysqlx/xdevapi.h
        PATHS ${MYSQLCONNECTORCPP_ROOT_DIR}/include
        )

find_library(MYSQLCONNECTORCPP_LIBRARY
        NAMES libmysqlcppconn8.so
        PATHS ${MYSQLCONNECTORCPP_ROOT_DIR}/lib ${MYSQLCONNECTORCPP_ROOT_DIR}/lib64
        )

mark_as_advanced(MYSQLCONNECTORCPP_INCLUDE_DIR MYSQLCONNECTORCPP_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MysqlConnectorCpp
        DEFAULT_MSG
        MYSQLCONNECTORCPP_INCLUDE_DIR
        MYSQLCONNECTORCPP_LIBRARY)

if(MYSQLCONNECTORCPP_FOUND)
    set(MYSQLCONNECTORCPP_INCLUDE_DIRS "${MYSQLCONNECTORCPP_INCLUDE_DIR}") # Add any dependencies here
    set(MYSQLCONNECTORCPP_LIBRARIES "${MYSQLCONNECTORCPP_LIBRARY}") # Add any dependencies here
    mark_as_advanced(MYSQLCONNECTORCPP_ROOT_DIR)
    message("-- MySQL connector 8 found")
else()
    message(FATAL_ERROR "Please install the MySQL C++ Connector version 8+ from \
     https://dev.mysql.com/doc/connector-cpp/8.0/en/connector-cpp-installation-source.html \
     by following the default settings (i.e., do NOT modify the install path).")
endif()

include_directories(${BOOST_INCLUDE_DIRS} ${MYSQLCONNECTORCPP_INCLUDE_DIR}/mysqlx)

set(SOURCES
    include/ades.h
    src/adesdb.cpp
    include/adesdb.h
    src/ades.cpp
    src/action_container.cpp
    include/action_container.h
    src/motion_container.cpp
    include/motion_container.h
    src/hades.cpp
    include/hades.h
    include/effect_model.h
    src/effect_model.cpp
    )

set(LIBRARIES
    ${BOOST_LIBRARIES}
    ${MYSQLCONNECTORCPP_LIBRARY}
    )

# Extension for DMPLib
find_package(DMP REQUIRED)
find_package(Simox QUIET)
find_package(Qt5OpenGL QUIET)
find_package(Eigen3 QUIET)

if(DMP_FOUND AND Simox_FOUND AND Eigen3_FOUND)
    set(SOURCES
        ${SOURCES}
        include/motion_containers/dmplib_motion.h
        src/motion_containers/dmplib_motion.cpp
        )

    set(LIBRARIES
        ${LIBRARIES}
        ${DMP_LIBS}
        ${Simox_LIBRARIES}
        ${Qt5_LIBRARIES}
        )

    include_directories(${DMP_INCLUDE_DIRS})
    include_directories(${EIGEN3_INCLUDE_DIR})

    add_executable(dmplib_motion_test src/test/dmplib_motion_test.cpp)
    target_link_libraries(dmplib_motion_test ${LIBRARIES} ades2)
endif()


add_library(ades2 SHARED ${SOURCES})
target_link_libraries(ades2 ${LIBRARIES})
