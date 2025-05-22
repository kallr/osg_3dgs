cmake_minimum_required(VERSION 3.10)
#project name

project(osg_3dgs)
SET(CMAKE_BUILD_TYPE "Release")

add_executable(${PROJECT_NAME}
                main.cpp
)
include_directories(usr/local/include)
link_directories(usr/local/lib)
target_link_libraries(${PROJECT_NAME}
                    osg
                    osgViewer
                    osgDB
)