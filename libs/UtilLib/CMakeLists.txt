 # Copyright (c) 2013 David Sichau <mail"at"sichau"dot"eu>
 #               2014 Simon Tanaka <tanakas"at"gmx"dot"ch>
 #
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 #
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.

##############################################################################
# include the header files
##############################################################################

#one below that UtilLib is included in the paths they are needed for the cmake config files
INCLUDE_DIRECTORIES(
   ${CMAKE_CURRENT_SOURCE_DIR}/../
   ${CMAKE_CURRENT_BINARY_DIR}/../
)

##############################################################################
# add the source files
##############################################################################
set(UTILLIB_SRCS

    src/geometry/Rectangle.cpp
    src/Exception.cpp
    src/ProgressBar.cpp
    src/MPIProxy.cpp
    src/Log.cpp

)

set(UTILLIB_HEADER

    include/geometry/QuadTreeNode.hpp
    include/geometry/QuadTree.hpp
    include/geometry/Rectangle.hpp
    include/geometry/FastNeighborList.hpp

    include/Exception.hpp
    include/Factory.hpp
    include/ProgressBar.hpp
    include/TerminalColors.hpp
    include/MPIProxy.hpp
    include/Timer.hpp
    include/Log.hpp
    include/Singleton.hpp

)

#############################################################################
# Libraries
#############################################################################
if (UNIX)
   ADD_DEFINITIONS("-fno-strict-aliasing" )
endif (UNIX)
set ( LIBLIST
    ${MPI_LIBRARIES}
    ${Boost_LIBRARIES}
)

add_library( ${LBMSOLVER_LIBRARY_PREFIX}UtilLib ${LIB_TYPE} ${UTILLIB_SRCS} ${UTILLIB_HEADER})

target_link_libraries( ${LBMSOLVER_LIBRARY_PREFIX}UtilLib ${LIBLIST} )

#############################################################################
# Cmake generated header files
#############################################################################
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.hpp.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/config.hpp
)


#############################################################################
# testing
#############################################################################

IF(BUILD_TESTS)
        ADD_SUBDIRECTORY(test)
ENDIF(BUILD_TESTS)



install( TARGETS ${LBMSOLVER_LIBRARY_PREFIX}UtilLib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib )
