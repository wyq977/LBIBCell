# - Enable Code Coverage
#
# 2012-01-31, Lars Bilke
#
# USAGE:
# 1. Copy this file into your cmake modules path
# 2. Add the following line to your CMakeLists.txt:
#      INCLUDE(CodeCoverage)
#
# 3. Use the function SETUP_TARGET_FOR_COVERAGE to create a custom make target
#    which runs your test executable and produces a lcov code coverage report.
#

# Check prereqs
FIND_PROGRAM( GCOV_PATH gcov )
FIND_PROGRAM( LCOV_PATH lcov )
FIND_PROGRAM( GENHTML_PATH genhtml )
FIND_PROGRAM( GCOVR_PATH gcovr PATHS ${CMAKE_SOURCE_DIR}/cmake-modules/scripts)

IF(NOT GCOV_PATH)
	MESSAGE(FATAL_ERROR "gcov not found! Aborting...")
ENDIF() # NOT GCOV_PATH

IF(NOT CMAKE_COMPILER_IS_GNUCXX)
if(NOT "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
	MESSAGE(FATAL_ERROR "Compiler is not GNU gcc or clang! Aborting...")
endif()
ENDIF() # NOT CMAKE_COMPILER_IS_GNUCXX

IF ( NOT CMAKE_BUILD_TYPE STREQUAL "Debug" )
  MESSAGE( WARNING "Code coverage results with an optimised (non-Debug) build may be misleading" )
ENDIF() # NOT CMAKE_BUILD_TYPE STREQUAL "Debug"


# Setup compiler options
ADD_DEFINITIONS(-fprofile-arcs -ftest-coverage)
IF(CMAKE_COMPILER_IS_GNUCXX)
LINK_LIBRARIES(gcov)
ENDIF()
if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
    ADD_LIBRARY(libprofile_rt STATIC IMPORTED)
    SET_TARGET_PROPERTIES(libprofile_rt PROPERTIES
        IMPORTED_LOCATION /usr/lib/libprofile_rt.a)
    LINK_LIBRARIES(libprofile_rt)
endif()

# Param _targetname     The name of new the custom make target
# Param _testrunner     The name of the target which runs the tests
# Param _outputname     lcov output is generated as _outputname.info
#                       HTML report is generated in _outputname/index.html
# Optional fourth parameter is passed as arguments to _testrunner
#   Pass them in list form, e.g.: "-j;2" for -j 2
FUNCTION(SETUP_TARGET_FOR_COVERAGE _targetname _testrunner _outputname)

	IF(NOT LCOV_PATH)
		MESSAGE(FATAL_ERROR "lcov not found! Aborting...")
	ENDIF() # NOT LCOV_PATH

	IF(NOT GENHTML_PATH)
		MESSAGE(FATAL_ERROR "genhtml not found! Aborting...")
	ENDIF() # NOT GENHTML_PATH

	# Setup target
	ADD_CUSTOM_TARGET(${_targetname}

		# Cleanup lcov
		COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --zerocounters --gcov-tool /opt/local/bin/gcov-mp-4.7


		# Run tests
		COMMAND  ${_testrunner} ${ARGV3}

		# Capturing lcov counters and generating report
		COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR} --capture --output-file ${_outputname}.info --gcov-tool /opt/local/bin/gcov-mp-4.7 --quiet >& temp.txt
		COMMAND ${LCOV_PATH} --remove ${_outputname}.info '*gcc47*' '*boost*' 'test/*' '/usr/*' --output-file ${_outputname}.info.cleaned --quiet  >& temp.txt
		COMMAND ${GENHTML_PATH}  --demangle-cpp -o ${_outputname} ${_outputname}.info.cleaned  --quiet >& temp.txt
		COMMAND ${CMAKE_COMMAND} -E remove ${_outputname}.info ${_outputname}.info.cleaned temp.txt

		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "Resetting code coverage counters to zero.\nProcessing code coverage counters and generating report."
	)

	# Show info where to find the report
	ADD_CUSTOM_COMMAND(TARGET ${_targetname} POST_BUILD
		COMMAND ;
		COMMENT "Open ./${_outputname}/index.html in your browser to view the coverage report."
	)

ENDFUNCTION() # SETUP_TARGET_FOR_COVERAGE

# Param _targetname     The name of new the custom make target
# Param _testrunner     The name of the target which runs the tests
# Param _outputname     cobertura output is generated as _outputname.xml
# Optional fourth parameter is passed as arguments to _testrunner
#   Pass them in list form, e.g.: "-j;2" for -j 2
FUNCTION(SETUP_TARGET_FOR_COVERAGE_COBERTURA _targetname _testrunner _outputname)

	IF(NOT PYTHON_EXECUTABLE)
		MESSAGE(FATAL_ERROR "Python not found! Aborting...")
	ENDIF() # NOT PYTHON_EXECUTABLE

	IF(NOT GCOVR_PATH)
		MESSAGE(FATAL_ERROR "gcovr not found! Aborting...")
	ENDIF() # NOT GCOVR_PATH

	ADD_CUSTOM_TARGET(${_targetname}

		# Run tests
		${_testrunner} ${ARGV3}

		# Running gcovr
		COMMAND ${GCOVR_PATH} -x -r ${CMAKE_SOURCE_DIR} -e '${CMAKE_SOURCE_DIR}/tests/'  -o ${_outputname}.xml
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		COMMENT "Running gcovr to produce Cobertura code coverage report."
	)

	# Show info where to find the report
	ADD_CUSTOM_COMMAND(TARGET ${_targetname} POST_BUILD
		COMMAND ;
		COMMENT "Cobertura code coverage report saved in ${_outputname}.xml."
	)

ENDFUNCTION() # SETUP_TARGET_FOR_COVERAGE_COBERTURA
