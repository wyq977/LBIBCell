##############################################################################
# include the header files
##############################################################################

#one below that GeometryLib is included in the paths they are needed for the cmake config files
INCLUDE_DIRECTORIES(
   ${CMAKE_CURRENT_SOURCE_DIR}/../
   ${CMAKE_CURRENT_BINARY_DIR}/../
)

##############################################################################
# add the source files
##############################################################################
set(LBMLIB_SRCS
        src/geometry/Geometry.cpp
        src/geometry/Connection.cpp
        src/geometry/GeometryHandler.cpp

        src/nodes/BoundaryNode.cpp
        src/nodes/EulerianPoint.cpp
        src/nodes/GeometryNode.cpp
        src/nodes/LagrangianPoint.cpp
        src/nodes/PhysicalNode.cpp

        src/reportHandler/DensityReporter.cpp
        src/reportHandler/DensityDomainReporter.cpp
        src/reportHandler/DomainReporter.cpp
        src/reportHandler/CellTypeReporter.cpp
        src/reportHandler/FluidReporter.cpp
        src/reportHandler/GridReporter.cpp
        src/reportHandler/ConnectionReporter.cpp
        src/reportHandler/ReportHandler.cpp
        src/reportHandler/SolverReporter.cpp
        src/reportHandler/UxReporter.cpp
        src/reportHandler/VelocityReporter.cpp
        src/reportHandler/MatixAbsVelocityReporter.cpp
        src/reportHandler/GeometryReporter.cpp
        src/reportHandler/vtkCellReporter.cpp
        src/reportHandler/vtkForceReporter.cpp
        src/reportHandler/vtkCDEReporter.cpp
        src/reportHandler/vtkFluidReporter.cpp
        src/reportHandler/vtkCellPNGReporter.cpp

        src/solver/BioSolver/BioSolverAreaRegulator.cpp
        src/solver/BioSolver/BioSolverCellDivision.cpp
        src/solver/BioSolver/BioSolverCellJunction.cpp
        src/solver/BioSolver/BioSolverDifferentiation.cpp
        src/solver/BioSolver/BioSolverEmpty.cpp
        src/solver/BioSolver/BioSolverGrowth.cpp
        src/solver/BioSolver/BioSolverMembraneTension.cpp
        src/solver/BioSolver/BioSolverHookeanMembraneTension.cpp
        src/solver/BioSolver/tutorial_01_BioSolverAreaRegulator.cpp
        src/solver/BioSolver/tutorial_01_BioSolverCellDivision.cpp
        src/solver/BioSolver/tutorial_01_BioSolverDifferentiation.cpp
        src/solver/BioSolver/tutorial_01_BioSolverMembraneTension.cpp
        src/solver/BioSolver/tutorial_01_BioSolverCellJunction.cpp
        src/solver/BioSolver/tutorial_02_BioSolverGrowth.cpp
        src/solver/BioSolver/tutorial_02_BioSolverMembraneTension.cpp
        src/solver/BioSolver/tutorial_02_BioSolverCellJunction.cpp
        src/solver/BioSolver/tutorial_02_BioSolverCellDivision.cpp
        src/solver/BioSolver/BioSolverRemoveCells.cpp
        src/solver/BioSolver/BioSolverCellDivisionRD.cpp

        src/solver/CDESolver/CDEAbstractSolver.cpp
        src/solver/CDESolver/CDESolverD2Q4.cpp
        src/solver/CDESolver/CDESolverD2Q5.cpp
        src/solver/CDESolver/CDESolverD2Q5BMP.cpp
        src/solver/CDESolver/CDESolverD2Q5HH.cpp
        src/solver/CDESolver/DiracD2Q4.cpp
        src/solver/CDESolver/DiracD2Q5.cpp
        src/solver/CDESolver/SchnakenbergD2Q4u.cpp
        src/solver/CDESolver/SchnakenbergD2Q4v.cpp
        src/solver/CDESolver/SchnakenbergD2Q5u.cpp
        src/solver/CDESolver/SchnakenbergD2Q5v.cpp
        src/solver/CDESolver/tutorial_01_CDESolverD2Q5_SIGNAL.cpp
        src/solver/CDESolver/tutorial_02_CDESolverD2Q5_R.cpp
        src/solver/CDESolver/tutorial_02_CDESolverD2Q5_L.cpp
        src/solver/CDESolver/CDESolverD2Q5_SHH.cpp

        src/solver/MassSolver/MassSolver.cpp
        src/solver/MassSolver/MassSolverSingleLayer.cpp
        src/solver/MassSolver/MassSolverSchnakenberg.cpp
        src/solver/MassSolver/MassSolverSingleGrowingCell.cpp
        src/solver/MassSolver/MassSolverBoxOutlet.cpp
        src/solver/MassSolver/MassSolverNoFluxXYCDED2Q5.cpp
        src/solver/MassSolver/MassSolverBoxOutletNoCDE.cpp
        
        src/solver/AbstractSolver.cpp
        src/solver/BoundaryAbstractSolver.cpp
        src/solver/BoundarySolverNoFlux.cpp
        src/solver/BoundarySolverNoFluxD2Q5.cpp
        src/solver/FluidSolver/FluidSolver.cpp
        src/solver/FluidSolver/BaseForceModel.cpp
        src/solver/FluidSolver/GuoZhengShi2002ForceModel.cpp
        src/solver/FluidSolver/Luo1993ForceModel.cpp
        src/solver/ForceSolver.cpp

        src/Direction.cpp
        src/GlobalSimulationParameters.cpp
        src/SimulationRunner.cpp
)

set(LBMLIB_HEADER

        include/geometry/Geometry.hpp
        include/geometry/Connection.hpp
        include/geometry/GeometryHandler.hpp

        include/nodes/BoundaryNode.hpp
        include/nodes/EulerianPoint.hpp
        include/nodes/GeometryNode.hpp
        include/nodes/LagrangianPoint.hpp
        include/nodes/PhysicalNode.hpp

        include/reportHandler/AbstractReportFunctor.hpp
        include/reportHandler/DensityReporter.hpp
        include/reportHandler/DensityDomainReporter.hpp
        include/reportHandler/DomainReporter.hpp
        include/reportHandler/CellTypeReporter.hpp
        include/reportHandler/FluidReporter.hpp
        include/reportHandler/GridReporter.hpp
        include/reportHandler/ConnectionReporter.hpp
        include/reportHandler/ReportHandler.hpp
        include/reportHandler/UxReporter.hpp
        include/reportHandler/SolverReporter.hpp
        include/reportHandler/VelocityReporter.hpp
        include/reportHandler/MatixAbsVelocityReporter.hpp
        include/reportHandler/GeometryReporter.hpp
        include/reportHandler/vtkCellReporter.hpp
        include/reportHandler/vtkForceReporter.hpp
        include/reportHandler/vtkCDEReporter.hpp
        include/reportHandler/vtkFluidReporter.hpp
        include/reportHandler/vtkCellPNGReporter.hpp

        include/solver/BioSolver/BioAbstractSolver.hpp
        include/solver/BioSolver/BioBaseSolver.hpp
        include/solver/BioSolver/BioSolverAreaRegulator.hpp
        include/solver/BioSolver/BioSolverCellDivision.hpp
        include/solver/BioSolver/BioSolverCellJunction.hpp
        include/solver/BioSolver/BioSolverDifferentiation.hpp
        include/solver/BioSolver/BioSolverEmpty.hpp
        include/solver/BioSolver/BioSolverFactory.hpp
        include/solver/BioSolver/BioSolverGrowth.hpp
        include/solver/BioSolver/BioSolverMembraneTension.hpp
        include/solver/BioSolver/BioSolverHookeanMembraneTension.hpp
        include/solver/BioSolver/tutorial_01_BioSolverAreaRegulator.hpp
        include/solver/BioSolver/tutorial_01_BioSolverCellDivision.hpp
        include/solver/BioSolver/tutorial_01_BioSolverDifferentiation.hpp
        include/solver/BioSolver/tutorial_01_BioSolverMembraneTension.hpp
        include/solver/BioSolver/tutorial_01_BioSolverCellJunction.hpp
        include/solver/BioSolver/tutorial_02_BioSolverGrowth.hpp
        include/solver/BioSolver/tutorial_02_BioSolverMembraneTension.hpp
        include/solver/BioSolver/tutorial_02_BioSolverCellJunction.hpp
        include/solver/BioSolver/tutorial_02_BioSolverCellDivision.hpp
        include/solver/BioSolver/BioSolverRemoveCells.hpp
        include/solver/BioSolver/BioSolverCellDivisionRD.hpp

        include/solver/CDESolver/CDEAbstractSolver.hpp
        include/solver/CDESolver/CDESolverD2Q4.hpp
        include/solver/CDESolver/CDESolverD2Q5.hpp
        include/solver/CDESolver/CDESolverD2Q5BMP.hpp
        include/solver/CDESolver/CDESolverD2Q5HH.hpp
        include/solver/CDESolver/CDESolverFactory.hpp
        include/solver/CDESolver/DiracD2Q4.hpp
        include/solver/CDESolver/DiracD2Q5.hpp
        include/solver/CDESolver/SchnakenbergD2Q4u.hpp
        include/solver/CDESolver/SchnakenbergD2Q4v.hpp
        include/solver/CDESolver/SchnakenbergD2Q5u.hpp
        include/solver/CDESolver/SchnakenbergD2Q5v.hpp
        include/solver/CDESolver/tutorial_01_CDESolverD2Q5_SIGNAL.hpp
        include/solver/CDESolver/tutorial_02_CDESolverD2Q5_R.hpp
        include/solver/CDESolver/tutorial_02_CDESolverD2Q5_L.hpp
        include/solver/CDESolver/CDESolverD2Q5_SHH.hpp

        include/solver/MassSolver/MassAbstractSolver.hpp
        include/solver/MassSolver/MassSolver.hpp
        include/solver/MassSolver/MassSolverSingleLayer.hpp
        include/solver/MassSolver/MassSolverSchnakenberg.hpp
        include/solver/MassSolver/MassSolverBoxOutlet.hpp
        include/solver/MassSolver/MassSolverFactory.hpp
        include/solver/MassSolver/MassSolverSingleGrowingCell.hpp
        include/solver/MassSolver/MassSolverNoFluxXYCDED2Q5.hpp
        include/solver/MassSolver/MassSolverBoxOutletNoCDE.hpp

        include/solver/AbstractSolver.hpp
        include/solver/BoundaryAbstractSolver.hpp
        include/solver/BoundarySolverFactory.hpp
        include/solver/BoundarySolverNoFlux.hpp
        include/solver/BoundarySolverNoFluxD2Q5.hpp
        include/solver/FluidSolver/FluidSolver.hpp
        include/solver/FluidSolver/BaseForceModel.hpp
        include/solver/FluidSolver/GuoZhengShi2002ForceModel.hpp
        include/solver/FluidSolver/Luo1993ForceModel.hpp
        include/solver/ForceSolver.hpp
        include/solver/ForceStructs.hpp
        include/solver/AbstractForceStruct.hpp

        include/Constants.hpp
        include/Direction.hpp
        include/Field.hpp
        include/GlobalSimulationParameters.hpp
        include/SimulationRunner.hpp
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
    ${LBMSOLVER_LIBRARY_PREFIX}UtilLib
)

add_library( ${LBMSOLVER_LIBRARY_PREFIX}LbmLib ${LIB_TYPE} ${LBMLIB_SRCS} ${LBMLIB_HEADER})

target_link_libraries( ${LBMSOLVER_LIBRARY_PREFIX}LbmLib ${LIBLIST} )

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

IF(BUILD_TESTING)
        ADD_SUBDIRECTORY(test)
ENDIF(BUILD_TESTING)



install( TARGETS ${LBMSOLVER_LIBRARY_PREFIX}LbmLib
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib )
