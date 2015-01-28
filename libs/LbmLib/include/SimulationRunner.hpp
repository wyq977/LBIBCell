/* Copyright (c) 2013       David Sichau <mail"at"sichau"dot"eu>
 *               2013-2015  Simon Tanaka <tanakas"at"gmx"dot"ch>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef SIMULATIONRUNNER_HPP
#define SIMULATIONRUNNER_HPP

#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/solver/ForceSolver.hpp>
#include <LbmLib/include/solver/MassSolver/MassAbstractSolver.hpp>
#include <LbmLib/include/solver/BioSolver/BioAbstractSolver.hpp>
#include <string>
#include <unordered_set>

namespace LbmLib {
namespace geometry {
class GeometryHandler;
}
namespace reportHandler {
class ReportHandler;
}

/**
 * @brief the main simulation class
 */
class SimulationRunner {
 public:
    /**
     * @brief constructor
     *
     * @param geometry the geometry handler
     * @param reportHandler the report handler
     */
    explicit SimulationRunner(
            const geometry::GeometryHandler& geometry,
            const reportHandler::ReportHandler& reportHandler);

    /**
     * @brief Destructor
     */
    ~SimulationRunner();

    /**
     * @brief runSimulation Runs the main Simulation loop
     */
    void runSimulation();

    /**
     * @brief writes the simulation to the files
     * @param forceFileName the name of the file where the forces are stored
     * @param geometryFileName the name of the file where the geometry is stored
     * @param parameterFileName the filename where the parameters are stored
     * @param solverFileName the name of the file wher the solvers are stored
     */
    void writeSimulation(
            const std::string& forceFileName,
            const std::string& geometryFileName,
            const std::string& parameterFileName,
            const std::string& solverFileName) const;

    /**
     * @brief initSolvers Initialises the solvers
     * @param fluidVelocity The initial Speed of the fluid
     */
    void initSolvers(Field<double> fluidVelocity);

    /**
     * @brief initSolvers Initialises the solvers
     */
    void initSolvers();

    /**
     * @brief Initialised the Solvers from a file
     * @param filename the file name where the solvers are stored
     */
    void initSolvers(const std::string& filename);

    /**
     * @brief adds a mass solver to the simulation
     * @param name the name of the mass solver
     */
    void addMassSolver(const std::string& name);

    /**
     * @brief Adds a BioSolver.
     * @param name The name of the BioSolver.
     */
    void addBioSolver(const std::string& name);

    /**
     * @brief Inits the Force solver to calculate the forces on the geometry nodes
     *
     * @param fileName The name of the Force solver input file
     */
    void initForceSolver(const std::string& fileName);


    /**
     * @brief getForceSolver Returns a reference to the solver::ForceSolver
     * @return reference to the solver::ForceSolver
     */
    const solver::ForceSolver& getForceSolver(void);

 private:
    /**
     * @brief setTau Set the Tau of a CDE Solver
     * @param tau The new tau
     * @param name The name of the CDE Solver
     */
    void setTau(
            double tau,
            const std::string& name="");

    /**
     * @brief addCDESolver Adds a CDESolver to all physical Nodes
     * @param name The name of the CDESolver
     */
    void addCDESolver(const std::string& name);

    /**
     * @brief calculates the force on the geometry nodes.
     */
    void calculateForce();

    /**
     * @brief collectVelocity collects the velocities from the fluid nodes to the geometry Nodes
     */
    void collectVelocity();

    /**
     * @brief distributeForce distributes the Forces from the geometry nodes to the fluid nodes
     */
    void distributeForce();

    /**
     * @brief moveGeometry Updates the Lattice after the Geometric Points have been moved
     */
    void moveGeometry();

    /**
     * @brief If a connection is too long, a GeometryNode is added and linked
     */
    void remeshBoundary();

    /**
     * @brief If a connections are too short, a GeometryNode is added removed.
     */
    void coarsenBoundary(void);

    /**
     * @brief Check lattice inegrity
     */
    void checkLatticeIntegrity();

    /**
     * @brief checkBoundaryIntegrity Checks whether the boundary nodes are set correctly.
     */
    void checkBoundaryIntegrity();

    /**
     * @brief collide Executes the collide step on all node
     */
    void collide();

    /**
     * @brief advect Executes the advect step on all node
     */
    void advect();

    /**
     * @brief preAdvect Executes the pre advect on the boundary nodes
     */
    void preAdvect();

    /**
     * @brief postAdvect Executes the post advect on the boundary nodes
     */
    void postAdvect();

    /**
     * @brief Applies all registered MassSolvers.
     */
    void applyMassSolvers();

    /**
     * @brief Applies all registered BioSolvers.
     */
    void applyBioSolvers();

    /**
     * @brief The force solver
     */
    solver::ForceSolver forceSolver_;

    /**
     * @brief forceSolver_ The mass solver reference.
     */
    std::vector<solver::MassAbstractSolver*> massSolvers_;

    /**
     * @brief The new BioSolvers.
     */
    std::vector<solver::BioAbstractSolver*> bioSolvers_;

    /**
     * @brief The geometry
     */
    const geometry::GeometryHandler& geometryHandler_;

    /**
     * @brief The Report Handler which controls the report Handlers
     */
    const reportHandler::ReportHandler& reportHandler_;
};
}  // end namespace

#endif  // SIMULATIONRUNNER_HPP
