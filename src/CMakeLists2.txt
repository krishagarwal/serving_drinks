cmake_minimum_required(VERSION 2.8)
project(libkinectwrapper)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

message("PROJECT_NAME:  " ${PROJECT_NAME})
message("PROJECT_NAME_INCLUDE_DIRS:  " ${PROJECT_NAME}_INCLUDE_DIRS)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(k4a REQUIRED)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

include_directories(
    include
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${VISION_GEOMETRY_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

SET(LIB_KINECT_WRAPPER_SRC
    src/SKConfig.cpp
    src/SKPacket.cpp
    src/SKPVideoDisplay.cpp
    src/SKWrapper.cpp
)

SET(LIB_KINECT_WRAPPER_INCLUDES
)


add_library(kinectwrapper ${LIB_KINECT_WRAPPER_SRC})

install(TARGETS kinectwrapper DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/kinectwrapper)