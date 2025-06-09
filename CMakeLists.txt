cmake_minimum_required (VERSION 3.12)

# 项目信息
set (PROJECT_NAME "osg_gaussian_draw")
project (${PROJECT_NAME})

# c++ 11
set (CMAKE_CXX_STANDARD 17)

SET (OSG_DEBUG_POSTFIX "d")

# qt
set (CMAKE_AUTOMOC ON)
set (CMAKE_AUTORCC ON)
set (CMAKE_AUTOUIC ON)


# 自动寻找QT5
find_package (Qt5 COMPONENTS Widgets Core Gui OpenGL REQUIRED)


#处理osg
IF(OSG_ROOT)
    SET(OSG_INCLUDE_DIR       "${OSG_ROOT}/include")
    SET(OSG_BUILD_INCLUDE_DIR "${OSG_ROOT}/include")
    SET(OSG_DEBUG_LIB_DIR     "${OSG_ROOT}/debug/lib")
    SET(OSG_RELEASE_LIB_DIR   "${OSG_ROOT}/release/lib")

ELSE(OSG_ROOT)
    FIND_PATH(OSG_INCLUDE_DIR osg/Referenced
        $ENV{OSG_ROOT}/include
        $ENV{OSG_DIR}/include
        /usr/include
        /usr/local/include
    )

    FIND_PATH(OSG_BUILD_INCLUDE_DIR osg/Config
        $ENV{OSG_ROOT}/include
        $ENV{OSG_DIR}/include
        /usr/include
        /usr/local/include
    )

    FIND_PATH(OSG_DEBUG_LIB_DIR
        NAMES libosgd.a libosgd.dll.a libosgd.dylib libosgd.so osgd.lib
        PATHS ${THIRDPARTY_ROOT}/lib
        $ENV{OSG_ROOT}/lib $ENV{OSG_ROOT}/${FIND_LIB_POSTFIX}
        $ENV{OSG_DIR}/lib $ENV{OSG_DIR}/${FIND_LIB_POSTFIX}
        ${OSG_BUILD_INCLUDE_DIR}/../lib
        ${OSG_BUILD_INCLUDE_DIR}/../debug/lib
        /usr/lib /usr/${FIND_LIB_POSTFIX} /usr/lib/${FIND_SUBLIB_POSTFIX}
        /usr/local/lib /usr/local/${FIND_LIB_POSTFIX}
    )
    FIND_PATH(OSG_RELEASE_LIB_DIR
        NAMES libosg.a libosg.dll.a libosg.dylib libosg.so osg.lib
        PATHS ${THIRDPARTY_ROOT}/lib
        $ENV{OSG_ROOT}/lib $ENV{OSG_ROOT}/${FIND_LIB_POSTFIX}
        $ENV{OSG_DIR}/lib $ENV{OSG_DIR}/${FIND_LIB_POSTFIX}
        ${OSG_BUILD_INCLUDE_DIR}/../lib
        ${OSG_BUILD_INCLUDE_DIR}/../release/lib
        /usr/lib /usr/${FIND_LIB_POSTFIX} /usr/lib/${FIND_SUBLIB_POSTFIX}
        /usr/local/lib /usr/local/${FIND_LIB_POSTFIX}
    )
ENDIF(OSG_ROOT)


FIND_PATH(GLM_DIR 
    NAMES glm/glm.hpp
    HINTS ${PROJECT_SOURCE_DIR}/vac/util ${PROJECT_SOURCE_DIR}/app1
)

find_path(GLM_DIR 
    NAMES gather.h
    HINTS ${PROJECT_SOURCE_DIR}/vac/util ${PROJECT_SOURCE_DIR}/app1
)
 

# include
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GLM_DIR}
    ${OSG_INCLUDE_DIR}
    ${OSG_BUILD_INCLUDE_DIR}
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
 

MACRO(LINK_OSG_LIBRARY PROJECT_NAME)
    FOREACH(OSG_LIB_NAME ${ARGN})
        STRING(FIND ${OSG_LIB_NAME} "OpenThreads" FOUND_OT_INDEX)
        IF(FOUND_OT_INDEX GREATER "-1")
            TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                debug     ${OSG_DEBUG_LIB_DIR}/${OSG_LIB_NAME}${OSG_DEBUG_POSTFIX}.lib
                optimized ${OSG_RELEASE_LIB_DIR}/${OSG_LIB_NAME}.lib)
        ELSE()
            TARGET_LINK_LIBRARIES(${PROJECT_NAME}
                debug     ${OSG_DEBUG_LIB_DIR}/${OSG_LIB_NAME}${OSG_DEBUG_POSTFIX}.lib
                optimized ${OSG_RELEASE_LIB_DIR}/${OSG_LIB_NAME}.lib)
        ENDIF()
    ENDFOREACH()
ENDMACRO(LINK_OSG_LIBRARY)
LINK_OSG_LIBRARY(${PROJECT_NAME} osgDB osgUtil osgViewer osgGA osg )

# Debug/Release库后缀自动处理
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_BINARY_DIR}/bin
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/bin
    )
endif()
 