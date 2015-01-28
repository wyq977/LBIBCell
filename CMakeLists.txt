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

project( lbibcell )
cmake_minimum_required(VERSION 2.6)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake-modules )

#############################################################################
# include cmake scripts
#############################################################################
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake-modules )


# require out-of-source build
INCLUDE(require_out_of_src_build)
MACRO_REQUIRE_OUT_OF_SRC_BUILD("LBMSolber requires out-of-source builds.")

# default install prefix. can be set with ccmake
set( CMAKE_INSTALL_PREFIX "/usr/local" CACHE STRING "Install prefix." )


set( LBMSOLVER_LIBRARY_PREFIX "lbm" CACHE STRING "prefix string for all libs" )

##############################################################################
# compile flags
##############################################################################

#flag -save-temps=obj needed on Mac OS X to prevent removing of assemble code
set(CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -openmp -g -save-temps=obj -Wall -Wextra -Wno-unused-parameter -Wno-error=unused-parameter -O0 -pedantic")

#for cross compiling remove the -march=native flag!
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -openmp -g -march=native")
set(CMAKE_CXX_FLAGS_MINSIZEREL "${CMAKE_CXX_FLAGS_MINSIZEREL}")



##############################################################################
# cmake options
##############################################################################

OPTION(BUILD_TESTS "Compile tests" ON)
OPTION(ENABLE_MEMORY_TEST "Generate memory test" OFF)
OPTION(ENABLE_COVERAGE "Enable GCov coverage analysis (defines a 'coverage' target)" OFF)

set(DEBUGLEVEL "logDEBUG1")
add_definitions( -DDEBUGLEVEL=${DEBUGLEVEL})




#only make dynamic libraries otherwise the solvers are not registered in the factory
SET(LIB_TYPE SHARED)

##############################################################################
# enable OpenMP dependent build
##############################################################################
FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message(STATUS "OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else()
    message(FATAL_ERROR "OpenMP support is required to build the application")
endif()


 #   add the flags for C++11
if(CMAKE_COMPILER_IS_GNUCXX)
  execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
        OUTPUT_VARIABLE GCC_VERSION)
  if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7)
    message(STATUS "C++11 activated.")
    add_definitions("-std=gnu++11")
  else ()
        message(WARNING "C++11 NOT activated. If you get any compile errors please use a C++11 compatible compiler like GNU gcc 4.7")
  endif()
else(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    add_definitions("-std=c++11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
  message(WARNING "C++11 NOT activated. If you get any compile errors please use a C++11 compatible compiler like GNU gcc 4.7")
  add_definitions("-std=c++11")
endif(CMAKE_COMPILER_IS_GNUCXX)

##############################################################################
# find vtk
##############################################################################
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
if( VTK_FOUND )
   message(STATUS "vtk version " ${VTK_VERSION})
else ( VTK_FOUND )
   message(FATAL_ERROR "vtk is mandatory to build the libraries")
endif( VTK_FOUND )

##############################################################################
# find boost
##############################################################################
find_package( Boost 1.54.0 )
if( Boost_FOUND )
   message(STATUS "Boost found. Setting up directories...")
   message(STATUS "BOOST_ROOT: " ${BOOST_ROOT} )
   message(STATUS "Boost include directories:" ${Boost_INCLUDE_DIR} )
   message(STATUS "Boost library directories:" ${Boost_LIBRARY_DIRS} )
   include_directories( ${Boost_INCLUDE_DIR} )
   link_directories( ${Boost_LIBRARY_DIR} )
else ( Boost_FOUND )
   message(FATAL_ERROR "Boost is mandatory to build the libraries")
endif( Boost_FOUND )




#############################################################################
# documentation
#############################################################################
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/doc/Doxyfile.cmake
${CMAKE_BINARY_DIR}/doc/Doxyfile
@ONLY IMMEDIATE
)

FIND_PACKAGE(Doxygen)

# target for doxygen
ADD_CUSTOM_TARGET(doc COMMAND ${DOXYGEN} ${CMAKE_BINARY_DIR}/doc/Doxyfile)


#############################################################################
# Test enabled
#############################################################################
IF(BUILD_TESTS)
  MESSAGE(STATUS "Testing enabled")
  FIND_PACKAGE( Boost COMPONENTS unit_test_framework REQUIRED )
  INCLUDE(CTest)
  ENABLE_TESTING()
ELSE(BUILD_TESTS)
  MESSAGE(STATUS "Testing disabled. To enable testing set BUILD_TESTS to true")
ENDIF(BUILD_TESTS)

#############################################################################
# code coverage analysis
#############################################################################

IF(ENABLE_COVERAGE AND BUILD_TESTS)
    INCLUDE(CodeCoverage)
    SETUP_TARGET_FOR_COVERAGE(coverage "make;test" coverage )
    MESSAGE(STATUS "Coverage targed is enabled run make coverage for generating a coverage report")
ELSE(ENABLE_COVERAGE AND BUILD_TESTS)
    IF(NOT BUILD_TESTING)
        MESSAGE(WARNING "Code coverages needs that testing is enabled")
    ELSE()
        MESSAGE(STATUS "Coverage disabled. To enable coverage set ENABLE_COVERAGE to true")
    ENDIF()
ENDIF(ENABLE_COVERAGE AND BUILD_TESTS)


include_directories( ./libs )
include_directories( ${CMAKE_SOURCE_DIR}/apps )
message("========================================libs==============================================")
ADD_SUBDIRECTORY( ./libs )
message("========================================apps==============================================")
ADD_SUBDIRECTORY( ./apps )
message("========================================Build Settings====================================")
MESSAGE(STATUS "Build type: " ${CMAKE_BUILD_TYPE})
MESSAGE(STATUS "Library Type: " ${LIB_TYPE})
MESSAGE(STATUS "Compiler flags:" ${CMAKE_CXX_COMPILE_FLAGS})
MESSAGE(STATUS "Compiler cxx debug flags:" ${CMAKE_CXX_FLAGS_DEBUG})
MESSAGE(STATUS "Compiler cxx release flags:" ${CMAKE_CXX_FLAGS_RELEASE})
MESSAGE(STATUS "Compiler cxx min size flags:" ${CMAKE_CXX_FLAGS_MINSIZEREL})
MESSAGE(STATUS "Compiler cxx flags:" ${CMAKE_CXX_FLAGS})
MESSAGE(STATUS "Log level:" ${DEBUGLEVEL})
message("========================================Build Settings finished===========================")

