INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_ELAD elad)

FIND_PATH(
    ELAD_INCLUDE_DIRS
    NAMES elad/api.h
    HINTS $ENV{ELAD_DIR}/include
        ${PC_ELAD_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    ELAD_LIBRARIES
    NAMES gnuradio-elad
    HINTS $ENV{ELAD_DIR}/lib
        ${PC_ELAD_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ELAD DEFAULT_MSG ELAD_LIBRARIES ELAD_INCLUDE_DIRS)
MARK_AS_ADVANCED(ELAD_LIBRARIES ELAD_INCLUDE_DIRS)

