#
# configure/build miind mpi tests
#
##############################################################################
# check if have valgrind
##############################################################################
IF(ENABLE_MEMORY_TEST)
    SET(HAVE_VALGRIND TRUE)
    FIND_PROGRAM(VALGRIND_EXECUTABLE valgrind)
    IF(VALGRIND_EXECUTABLE STREQUAL "VALGRIND_EXECUTABLE-NOTFOUND")
        MESSAGE(STATUS "Error: Could not find valgrind. For memory test valgrind is needed.")
        SET(HAVE_VALGRIND FALSE)
    ENDIF(VALGRIND_EXECUTABLE STREQUAL "VALGRIND_EXECUTABLE-NOTFOUND")

    SET(Test_Dir ${CMAKE_CURRENT_SOURCE_DIR})
ENDIF(ENABLE_MEMORY_TEST)


##############################################################################
# check if MPI Executable is available
##############################################################################
SET(HAVE_MPIRUN TRUE)
IF(MPIEXEC STREQUAL "MPIEXEC-NOTFOUND")
    MESSAGE(FATAL_ERROR "ERROR: Could not find mpirun. Will NOT build tests. Either deactivate test or provide the variable MPIEXEC")
    SET(HAVE_MPIRUN FALSE)
ENDIF(MPIEXEC STREQUAL "MPIEXEC-NOTFOUND")

##############################################################################
# macro definitions
##############################################################################

# MPI test that is only executed if MPI is enabled
MACRO(ADD_MPI_TEST src)
    IF(ENABLE_MPI)


        #make sure that the tests all end with _test.cpp
        STRING(REGEX MATCH "_test.cpp$" result ${src})
        IF(NOT(${result} MATCHES "_test.cpp"))
            MESSAGE(FATAL_ERROR "test need to end with _test.cpp")
        ENDIF()
        #set the test name by removing the _test.cpp from the source file
        STRING(REGEX REPLACE "_test.cpp" "" name ${src})

        #set the name of executable by prepending test_ to the beginning
        SET(exe "test_" ${name})
        #and replace all / with _
        STRING(REGEX REPLACE "/" "_" exe ${exe})


        # build the test
        ADD_EXECUTABLE(${exe} ${src})

        SET(TEST_LIBS
            ${LBMSOLVER_LIBRARY_PREFIX}UtilLib
            ${MPI_LIBRARIES}
            ${Boost_LIBRARIES}
        )

        TARGET_LINK_LIBRARIES(${exe} ${TEST_LIBS})


        ADD_TEST(${name} ${MPIEXEC} -np 2 ${exe})
        MESSAGE(STATUS "Adding mpi unit test for: ${name}: ${exe}.")
        # add target for the test
        STRING(REGEX REPLACE "test_([^ ]+).*" "unit_\\1" unittest_target "${exe}" )
        ADD_CUSTOM_TARGET(${unittest_target} COMMAND mpirun -np 2 ${CMAKE_CURRENT_BINARY_DIR}/${exe})

    ELSE(ENABLE_MPI)
         ADD_TEST_FILE(${src})
    ENDIF(ENABLE_MPI)
ENDMACRO(ADD_MPI_TEST src)

# MPI test that is always executed
MACRO(ADD_TEST_FILE src)

    #make sure that the tests all end with _test.cpp
    STRING(REGEX MATCH "_test.cpp$" result ${src})
    IF(NOT(${result} MATCHES "_test.cpp"))
        MESSAGE(FATAL_ERROR "test need to end with _test.cpp")
    ENDIF()

    #set the test name by removing the _test.cpp from the source file
    STRING(REGEX REPLACE "_test.cpp" "" name ${src})

    #set the name of executable by prepending test_ to the beginning
    SET(exe "test_" ${name})
    #and replace all / with _
    STRING(REGEX REPLACE "/" "_" exe ${exe})

    # build the test
    ADD_EXECUTABLE(${exe} ${src})

    SET(TEST_LIBS
        ${LBMSOLVER_LIBRARY_PREFIX}UtilLib
        ${Boost_LIBRARIES}
    )

    TARGET_LINK_LIBRARIES(${exe} ${TEST_LIBS})

    ADD_TEST(${name} ${exe})
    MESSAGE(STATUS "Adding unit test for: ${name}: ${exe}.")
    # add target for the test
    STRING(REGEX REPLACE "test_([^ ]+).*" "unit_\\1" unittest_target "${exe}" )
    ADD_CUSTOM_TARGET(${unittest_target} COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${exe})

    IF (ENABLE_MEMORY_TEST)
        #add memory test using valgrind
        STRING(REGEX REPLACE "test_([^ ]+).*" "memtest_\\1" memtest_name "${exe}" )
        LIST(APPEND memtest_names ${memtest_name})
        LIST(APPEND exe_names ${exe})
        #Add target for the memory test
        ADD_CUSTOM_TARGET(${memtest_name} COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/memcheck.py ${CMAKE_CURRENT_BINARY_DIR}/${exe})
        MESSAGE(STATUS "Adding memory test for:   ${name}: ${exe}.")
        ADD_TEST(${memtest_name}
           ${Test_Dir}/memtest.py ${CMAKE_CURRENT_BINARY_DIR}/${exe} ${CMAKE_BINARY_DIR})
    ENDIF (ENABLE_MEMORY_TEST)
ENDMACRO(ADD_TEST_FILE src)

##############################################################################
# include directories
##############################################################################
INCLUDE_DIRECTORIES(
    ../../LBMSolverLib/include
)




##############################################################################
# Add here the test the test source files need to end with _test.cpp
##############################################################################

message(STATUS "=== Unit tests UtilLib ===")


ADD_TEST_FILE(Exception_test.cpp)
ADD_TEST_FILE(Factory_test.cpp)
ADD_MPI_TEST(Log_test.cpp)
ADD_MPI_TEST(MPIProxy_test.cpp)
ADD_TEST_FILE(Singleton_test.cpp)
ADD_MPI_TEST(ProgressBar_test.cpp)
ADD_TEST_FILE(geometry/Rectangle_test.cpp)
ADD_TEST_FILE(geometry/FastNeighborList_test.cpp)


