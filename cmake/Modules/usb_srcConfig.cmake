INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_USB_SRC usb_src)

FIND_PATH(
    USB_SRC_INCLUDE_DIRS
    NAMES usb_src/api.h
    HINTS $ENV{USB_SRC_DIR}/include
        ${PC_USB_SRC_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    USB_SRC_LIBRARIES
    NAMES gnuradio-usb_src
    HINTS $ENV{USB_SRC_DIR}/lib
        ${PC_USB_SRC_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(USB_SRC DEFAULT_MSG USB_SRC_LIBRARIES USB_SRC_INCLUDE_DIRS)
MARK_AS_ADVANCED(USB_SRC_LIBRARIES USB_SRC_INCLUDE_DIRS)

