# Copyright 2011,2012 Free Software Foundation, Inc.
#
# This file is part of GNU Radio
#
# GNU Radio is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
#
# GNU Radio is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with GNU Radio; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.

########################################################################
# Setup library
########################################################################
include(GrPlatform) #define LIB_SUFFIX

include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

list(APPEND elad_sources
    fdm_source_c_impl.cc
)

set(elad_sources "${elad_sources}" PARENT_SCOPE)
if(NOT elad_sources)
	MESSAGE(STATUS "No C++ sources... skipping lib/")
	return()
endif(NOT elad_sources)

add_library(gnuradio-elad SHARED ${elad_sources})
target_link_libraries(gnuradio-elad ${Boost_LIBRARIES} ${GNURADIO_ALL_LIBRARIES} usb-1.0)

set_target_properties(gnuradio-elad PROPERTIES DEFINE_SYMBOL "gnuradio_elad_EXPORTS")

if(APPLE)
    set_target_properties(gnuradio-elad PROPERTIES
        INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib"
    )
endif(APPLE)

########################################################################
# Install built library files
########################################################################
install(TARGETS gnuradio-elad
    LIBRARY DESTINATION lib${LIB_SUFFIX} # .so/.dylib file
    ARCHIVE DESTINATION lib${LIB_SUFFIX} # .lib file
    RUNTIME DESTINATION bin              # .dll file
)

