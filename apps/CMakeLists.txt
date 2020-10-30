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
# copy config and postprocessing to build
##############################################################################

ADD_SUBDIRECTORY( config )
ADD_SUBDIRECTORY( postprocessing )

file(COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/config/
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/config)

file(COPY
    ${CMAKE_CURRENT_SOURCE_DIR}/postprocessing/
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/postprocessing)



##############################################################################
# include the header files
##############################################################################

#include the MPILib build dir for the config file
INCLUDE_DIRECTORIES(
   ${CMAKE_CURRENT_BINARY_DIR}/../libs/
)


#############################################################################
# Libraries
#############################################################################

if (UNIX)
   ADD_DEFINITIONS("-fno-strict-aliasing" )
endif (UNIX)
set ( LIBLIST
#    ${MPI_LIBRARIES}
    ${LBMSOLVER_LIBRARY_PREFIX}LbmLib
    ${LBMSOLVER_LIBRARY_PREFIX}UtilLib
)

MACRO(ADD_APPLICATION src)

    #make sure that the tests all end with _test.cpp
    STRING(REGEX MATCH ".cpp$" result ${src})
    IF(NOT(${result} MATCHES ".cpp"))
        MESSAGE(FATAL_ERROR "application needs to end with .cpp")
    ENDIF()
    #make sure that the tests all end with _test.cpp
    STRING(REGEX MATCH ".cpp$" result ${src})

    #set the test name by removing the _test.cpp from the source file
    STRING(REGEX REPLACE ".cpp" "" exe ${src})

    # build the application
    ADD_EXECUTABLE(${exe} ${src})
    TARGET_LINK_LIBRARIES(${exe} ${LIBLIST} ${VTK_LIBRARIES})

    MESSAGE(STATUS "Adding application for: ${exe}.")
ENDMACRO(ADD_APPLICATION src)
#############################################################################
# executables
#############################################################################
ADD_APPLICATION(tutorial_01.cpp)
ADD_APPLICATION(tutorial_02.cpp)
ADD_APPLICATION(syn_tissue.cpp)
ADD_APPLICATION(syn_tissue_random_division.cpp)


