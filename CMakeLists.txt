cmake_minimum_required (VERSION 3.12)

# 项目信息
set (PROJECT_NAME "osg_gaussian_draw")
project (${PROJECT_NAME})

# c++ 11
set (CMAKE_CXX_STANDARD 17)


# qt
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTORCC ON)
set (CMAKE_AUTOUIC ON)


# 路径设置
if(WIN32)
    list(APPEND CMAKE_PREFIX_PATH "F:/QT/5.15.2/5.15.2/msvc2019_64")
    list(APPEND CMAKE_PREFIX_PATH "I:/thbim/3rd/OpenSceneGraph-3.6.5")
elseif(UNIX)
    list(APPEND CMAKE_PREFIX_PATH "/usr/local/QT")
endif()

# 自动寻找QT5
find_package (Qt5 COMPONENTS Widgets Core Gui OpenGL REQUIRED)

 # OSG
find_package(OpenThreads REQUIRED)
find_package(osg REQUIRED)
find_package(osgViewer REQUIRED)
find_package(osgDB REQUIRED)
find_package(osgGA REQUIRED)
find_package(osgUtil REQUIRED)

# include
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/glm
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
)
 
# 源文件
set (SRC_FILE
        main.cpp
        mainwindow.cpp
        osg/osgWindow.cpp
		osg/GaussianDrawObj.cpp
        tools/tools.cpp
		tools/miniply.cpp
)

# 头文件
set (HEADER_FILE
        mainwindow.h
        osg/osgWindow.h
		osg/GaussianDrawObj.h
        tools/tools.h
		tools/miniply.h
)
 

# shader
set(SHADER
gaussian.frag
gaussian.vert
point.vert
point.frag
)

#加入项目文件
add_executable (${PROJECT_NAME}
        ${SRC_FILE}
        ${HEADER_FILE}  
)

# 链接Qt
target_link_libraries(${PROJECT_NAME} Qt5::Widgets Qt5::Core Qt5::Gui)

# 链接OSG
target_link_libraries(${PROJECT_NAME}
    OpenThreads
    osg
    osgViewer
    osgDB
    osgGA
    osgUtil
)


# Debug/Release库后缀自动处理
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_BINARY_DIR}/bin/Debug
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin/Release
    )
endif()
 