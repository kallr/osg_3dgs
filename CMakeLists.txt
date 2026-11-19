cmake_minimum_required (VERSION 3.12)

# 项目信息
set (PROJECT_NAME "osg_gaussian_draw")
project (${PROJECT_NAME})

# c++ 17
set (CMAKE_CXX_STANDARD 17)

SET (OSG_DEBUG_POSTFIX "d")

option(OSG_3DGS_BUILD_GUI "Build the Qt test viewer" ON)
if(OSG_3DGS_BUILD_GUI)
    find_package(Qt5 REQUIRED COMPONENTS Widgets)
endif()
find_package(OpenGL REQUIRED)

# zlib-backed SPZ/RAD formats
option(OSG_3DGS_ENABLE_ZLIB_FORMATS "Enable zlib-backed SPZ/RAD support" ON)
find_package(ZLIB QUIET)

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

# GaussForge sources (multi-format gaussian splatting IO)
set(GAUSSFORGE_SRC
    3rdparty/GaussForge/registry.cpp
    3rdparty/GaussForge/ply_reader.cpp
    3rdparty/GaussForge/ply_auto.cpp
    3rdparty/GaussForge/ply_compressed_reader.cpp
    3rdparty/GaussForge/splat_reader.cpp
    3rdparty/GaussForge/ksplat_reader.cpp
    3rdparty/GaussForge/validate.cpp
)

# SPZ/RAD compressed formats require zlib headers and library.
set(HAS_GAUSSIAN_ZLIB OFF)
set(GAUSSIAN_ZLIB_TARGET "")
if(OSG_3DGS_ENABLE_ZLIB_FORMATS)
    if(NOT ZLIB_FOUND)
        find_package(Qt5 COMPONENTS Core Zlib QUIET)
    endif()
    if(ZLIB_FOUND)
        set(HAS_GAUSSIAN_ZLIB ON)
        set(GAUSSIAN_ZLIB_TARGET ZLIB::ZLIB)
    elseif(TARGET Qt5::Zlib)
        get_target_property(QT_ZLIB_INCLUDE_DIRS Qt5::Zlib INTERFACE_INCLUDE_DIRECTORIES)
        if(QT_ZLIB_INCLUDE_DIRS)
            set(HAS_GAUSSIAN_ZLIB ON)
            set(GAUSSIAN_ZLIB_TARGET Qt5::Zlib Qt5::Core)
        else()
            message(FATAL_ERROR "Qt5::Zlib was found but does not publish zlib include directories; install zlib or configure with -DOSG_3DGS_ENABLE_ZLIB_FORMATS=OFF to disable SPZ/RAD")
        endif()
    else()
        message(FATAL_ERROR "ZLIB not found; install zlib or configure with -DOSG_3DGS_ENABLE_ZLIB_FORMATS=OFF to disable SPZ/RAD")
    endif()
endif()

if(HAS_GAUSSIAN_ZLIB)
    list(APPEND GAUSSFORGE_SRC
        3rdparty/GaussForge/spz_reader.cpp
        3rdparty/spz/load-spz.cc
        3rdparty/spz/splat-types.cc
    )
endif()

# 源文件
set (CORE_SOURCES
    osg/GaussianModel.cpp
    osg/GaussianDrawObj.cpp
    osg/GaussianLODTree.cpp
    osg/SparkShaderLibrary.cpp
    osg/SparkOsgCallbacks.cpp
    osg/SparkDatasetLoader.cpp
    osg/SparkRadDatasetLoader.cpp
    osg/SparkRadFile.cpp
    osg/SparkRadStreamer.cpp
    osg/SparkLodSelector.cpp
    osg/SparkLodWorker.cpp
    osg/SparkGpuResources.cpp
    osg/SparkSortPass.cpp
    osg/SparkOsgDrawable.cpp
    osg/SparkTileBinner.cpp
    osg/SparkTileRasterPass.cpp
    osg/SparkCompositePass.cpp
    osg/SparkSplatEdit.cpp
    osg/SparkSplatSkinning.cpp
    osg/SparkPortal.cpp
    tools/miniply.cpp
    ${GAUSSFORGE_SRC}
)

# 头文件
set (CORE_HEADERS
    include/osg_3dgs/GaussianModel.h
    osg/GaussianDrawObj.h
    osg/GaussianLODTree.h
    osg/SparkShaderLibrary.h
    osg/SparkOsgCallbacks.h
    osg/SparkMaterialBuilder.h
    osg/SparkRenderTypes.h
    osg/SparkDatasetLoader.h
    osg/SparkRadDatasetLoader.h
    osg/SparkActiveSet.h
    osg/SparkRadFile.h
    osg/SparkRadStreamer.h
    osg/SparkRadProtectedPages.h
    osg/SparkLodSelector.h
    osg/SparkLodWorker.h
    osg/SparkGpuResources.h
    osg/SparkSortPass.h
    osg/SparkOsgDrawable.h
    osg/SparkTileBinner.h
    osg/SparkTileRasterPass.h
    osg/SparkCompositePass.h
    osg/SparkSplatEdit.h
    osg/SparkSplatSkinning.h
    osg/SparkPortal.h
    tools/miniply.h
)

add_library(osg_3dgs_core STATIC ${CORE_SOURCES} ${CORE_HEADERS})
add_library(osg_3dgs::core ALIAS osg_3dgs_core)
target_compile_features(osg_3dgs_core PUBLIC cxx_std_17)
target_include_directories(osg_3dgs_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OSG_INCLUDE_DIR}
        ${OSG_BUILD_INCLUDE_DIR}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/GaussForge
        ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/spz
)
set_target_properties(osg_3dgs_core PROPERTIES DEBUG_POSTFIX "d" POSITION_INDEPENDENT_CODE ON)
if(WIN32)
    set_target_properties(osg_3dgs_core PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_BINARY_DIR}/lib"
        ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/lib")
endif()

target_link_libraries(osg_3dgs_core PRIVATE OpenGL::GL)
if(HAS_GAUSSIAN_ZLIB)
    target_link_libraries(osg_3dgs_core PRIVATE ${GAUSSIAN_ZLIB_TARGET})
    target_compile_definitions(osg_3dgs_core PRIVATE HAVE_ZLIB=1 HAS_ZLIB=1)
endif()

FUNCTION(LINK_OSG_LIBRARY target visibility)
    FOREACH(OSG_LIB_NAME ${ARGN})
        TARGET_LINK_LIBRARIES(${target} ${visibility}
            debug     ${OSG_DEBUG_LIB_DIR}/${OSG_LIB_NAME}${OSG_DEBUG_POSTFIX}.lib
            optimized ${OSG_RELEASE_LIB_DIR}/${OSG_LIB_NAME}.lib)
    ENDFOREACH()
ENDFUNCTION()
LINK_OSG_LIBRARY(osg_3dgs_core PUBLIC osg OpenThreads)
LINK_OSG_LIBRARY(osg_3dgs_core PRIVATE osgDB osgUtil)

if(OSG_3DGS_BUILD_GUI)
    add_executable(osg_gaussian_draw main.cpp mainwindow.cpp mainwindow.h osg/osgWindow.cpp osg/osgWindow.h)
    set_target_properties(osg_gaussian_draw PROPERTIES
        AUTOMOC ON AUTORCC ON AUTOUIC ON DEBUG_POSTFIX "d")
    target_include_directories(osg_gaussian_draw PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(osg_gaussian_draw PRIVATE osg_3dgs::core Qt5::Widgets OpenGL::GL)
    LINK_OSG_LIBRARY(osg_gaussian_draw PRIVATE osgDB osgUtil osgViewer osgGA)
    if(WIN32)
        set_target_properties(osg_gaussian_draw PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG   "${CMAKE_CURRENT_BINARY_DIR}/bin"
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_CURRENT_BINARY_DIR}/bin")
    endif()

    add_custom_command(TARGET osg_gaussian_draw POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/shader"
        "$<TARGET_FILE_DIR:osg_gaussian_draw>/shader"
        COMMENT "Copying shader files to output directory"
    )
endif()
