cmake_minimum_required(VERSION 3.9)
project(ProbabilisticModelling)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp )

FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(OpenCV  REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

SET( BOOST_INCLUDEDIR /usr/local/boost_1_66_0)
FIND_PACKAGE(boost REQUIRED)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIRS})

add_executable(Model
        ${SOURCE_FILES}
        defs.hpp
        )

target_link_libraries(
        Model
        ${OpenCV_LIBS}
)
