
find_package(PkgConfig)
pkg_check_modules(
    MY_LIBUSB
    IMPORTED_TARGET
    libusb-1.0
)

## XS COMMON

file(
    GLOB XSCOMMON_SRC_FILES
    xscommon/*.c xscommon/*.cpp xscommon/*.h
)

add_library(
    xscommon
    STATIC
    ${XSCOMMON_SRC_FILES}
)

# Note: use 'SHELL:' prefix to prevent option de-duplication here:
target_compile_options(
    xscommon
    PRIVATE
    -fPIC
    "SHELL:-include xstypes/xsens_compat.h"
    "SHELL:-include xscommon_config.h"
)

set_target_properties(
    xscommon
    PROPERTIES
    CXX_STANDARD 11
)

target_include_directories(
    xscommon
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    xscommon
)

## XS CONTROLLER

file(
    GLOB XSCONTROLLER_SRC_FILES
    xscontroller/*.c xscontroller/*.cpp xscontroller/*.h
)

add_library(
    xscontroller
    STATIC
    ${XSCONTROLLER_SRC_FILES}
)

target_compile_options(
    xscontroller
    PRIVATE
    -fPIC
    "SHELL:-include xscontrollerconfig.h"
)

target_compile_definitions(
    xscontroller
    PRIVATE
    -DHAVE_JOURNALLER
)

if(${MY_LIBUSB_FOUND})
    target_compile_definitions(
        xscontroller
        PRIVATE
        -DHAVE_LIBUSB
    )
endif()

set_target_properties(
    xscontroller
    PROPERTIES
    CXX_STANDARD 11
)

target_include_directories(
    xscontroller
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    xscontroller
)

# XS TYPES

file(
    GLOB SRC_FILES
    xstypes/*.c xstypes/*.cpp xstypes/*.h
)

list(REMOVE_ITEM ${SRC_FILES} xstypes/xstypesdynlib.c)

add_library(
    xstypes
    STATIC
    ${SRC_FILES}
)

target_compile_options(
    xstypes
    PRIVATE
    -fPIC
)

set_target_properties(
    xstypes
    PROPERTIES
    CXX_STANDARD 11
)

target_include_directories(
    xstypes
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
