cmake_minimum_required(VERSION 2.8)
PROJECT( rotation )
FIND_PACKAGE( OpenCV REQUIRED )
ADD_EXECUTABLE( rotation rotation.cpp )
TARGET_LINK_LIBRARIES( rotation ${OpenCV_LIBS} )
SET(CMAKE_CXX_FLAGS "-std=c++0x")
