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
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>

#include <LbmLib/include/SimulationRunner.hpp>

#include <LbmLib/include/reportHandler/ReportHandler.hpp>
#include <LbmLib/include/solver/MassSolver/MassSolverFactory.hpp>

#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/solver/BoundaryAbstractSolver.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>
#include <LbmLib/include/solver/FluidSolver/FluidSolver.hpp>
#include <UtilLib/include/ProgressBar.hpp>
#include <omp.h>
#include <cassert>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <limits.h>

namespace {
unsigned int t; ///< The time increment. Need to make accessible to expection handler.
}

namespace LbmLib {
SimulationRunner::SimulationRunner(
        const geometry::GeometryHandler& geometry,
        const reportHandler::ReportHandler& reportHandler)
    :
      geometryHandler_(geometry),
      reportHandler_(reportHandler)
{
    this->setTau(Parameters.getTauFluid());
    const auto& cdeSolvers = Parameters.getCdeSolvers();
    for (const auto& cdeSolver : cdeSolvers)
    {
        this->addCDESolver(cdeSolver.first);
        this->setTau(cdeSolver.second, cdeSolver.first);
    }
}

SimulationRunner::~SimulationRunner()
{}

void SimulationRunner::runSimulation() {
    UtilLib::ProgressBar progress(Parameters.getIterations() - 1);
    try {
        for (t = 1; t < Parameters.getIterations()+1; t++) {
            this->calculateForce();
            this->distributeForce();
            this->preAdvect();
            this->advect();
            this->postAdvect();
            this->collide();
            this->collectVelocity();

            this->moveGeometry();

            this->remeshBoundary();
            if (Parameters.getCurrentIteration()%10 == 0) {
                this->forceSolver_.deleteAllForces();
                this->coarsenBoundary();
            }

            this->applyMassSolvers();
            this->applyBioSolvers();

#ifndef NDEBUG
            this->checkLatticeIntegrity();
            this->checkBoundaryIntegrity();
#endif

            progress++;
            Parameters++;
            this->reportHandler_.writeReport(t);
        }
    } catch(const std::exception& exp) {
        Parameters.setReportSteps(1);
        this->reportHandler_.writeCrashReport(t);
        this->writeSimulation("output/CRASH_forceOut_" + std::to_string(t) + ".txt",
                              "output/CRASH_geoOut_" + std::to_string(t) + ".txt",
                              "output/CRASH_parameterOut_" + std::to_string(t) + ".txt",
                              "output/CRASH_solverOut_" + std::to_string(t) + ".txt");
        std::cout << exp.what() << std::endl;
    }
}

void SimulationRunner::writeSimulation(
        const std::string& forceFileName,
        const std::string& geometryFileName,
        const std::string& parameterFileName,
        const std::string& solverFileName) const {
    this->forceSolver_.writeForceSolver(forceFileName);
    this->geometryHandler_.getGeometry().writeGeometry(geometryFileName);
    Parameters.writeGlobalSimulationParameters(parameterFileName);

    //loadable fulldump of the solvers:
    std::ofstream fileStream;
    fileStream.open(solverFileName);
    if (fileStream.is_open()) {
        fileStream << "#Solvers (xPos\tyPos\tDistributions...)\n";
        fileStream << "#BEGIN FluidSolver\n";
        for (auto it : geometryHandler_.getPhysicalNodes()) {
            for (auto i : it) {
                i->getFluidSolver().writeSolver(&fileStream);
            }
        }
        fileStream << "#END FluidSolver\n";
        for (auto cde : geometryHandler_.getPhysicalNodes()[0][0]->
               getCDESolvers()) {
            fileStream << "#BEGIN " << cde->getName() << "\n";
            for (auto it : geometryHandler_.getPhysicalNodes()) {
                for (auto i : it) {
                    i->getCDESolver(cde->getId()).writeSolver(&fileStream);
                }
            }
            fileStream << "#END " <<  cde->getName() << "\n";
        }
    } else {
        lbm_fail(
                "Cannot open the output file for the force solver.");
    }

    fileStream.close();
}

void SimulationRunner::initSolvers(const std::string& filename) {
    std::ifstream fileStream;
    fileStream.open(filename);
    if (fileStream.is_open()) {
        std::string line;
        std::string solverName("");
        while (std::getline(fileStream, line)) {
            std::stringstream lineStream(line);
            if (line.at(0) != '#') {
                int x, y;
                lineStream >> x >> y;
                if (solverName == "FluidSolver") {
                    geometryHandler_.getPhysicalNodes()[y][x]->getFluidSolver()
                      .loadSolver(&lineStream);
                } else {
                    // reset of the stringstream
                    lineStream.seekg(0);
                    geometryHandler_.getPhysicalNodes()[y][x]->getCDESolverSlow(
                            solverName).loadSolver(&lineStream);
                }
            } else if (line.find("#BEGIN") != std::string::npos) {
                std::string temp;
                lineStream >> temp >> solverName;
            } else if (line.find("#END") != std::string::npos) {
                solverName = "";
            }
        }
    } else {
        lbm_fail("Cannot find the force file.");
    }
    fileStream.close();
}

void SimulationRunner::moveGeometry() {
    //  We cast away constness to allow this function call where the Lattice might be changed.
    //  As this should not happens frequently we assume this as OK
    const_cast<geometry::GeometryHandler&>(this->geometryHandler_).moveLattice();
}

void SimulationRunner::remeshBoundary() {
    while ( // remesh as long as there is nothing to remesh any more
           //  We cast away constness to allow this function call where the Lattice might be changed.
           //  As this should not happens frequently we assume this as OK
           const_cast<geometry::GeometryHandler&>(this->geometryHandler_).remeshBoundary()
           )
    {
        assert(this->geometryHandler_.checkGeometryIntegrity());
    }
}

void SimulationRunner::coarsenBoundary()
{
    //  We cast away constness to allow this function call where the Lattice might be changed.
    //  As this should not happens frequently we assume this as OK
    const_cast<geometry::GeometryHandler&>(this->geometryHandler_).coarsenBoundary();
    assert(this->geometryHandler_.checkGeometryIntegrity());
}

void SimulationRunner::checkLatticeIntegrity()
{
    const_cast<geometry::GeometryHandler&>(this->geometryHandler_).checkLatticeIntegrity();
}

void SimulationRunner::checkBoundaryIntegrity()
{
    const_cast<geometry::GeometryHandler&>(this->geometryHandler_).checkBoundaryNodeIntegrity();
}


void SimulationRunner::collectVelocity() {
    typedef std::map<unsigned int,std::shared_ptr<nodes::GeometryNode> >::const_iterator MapIterator;
    std::vector<MapIterator> helper_vector;

    // create helper_vector for parallelzation:
    for (MapIterator it = this->geometryHandler_.getGeometry().getGeometryNodes().begin();
         it != this->geometryHandler_.getGeometry().getGeometryNodes().end();
         it++)
    {
        helper_vector.push_back(it);
    }

    // collect velocity:
#pragma omp parallel for schedule(static)
    for (size_t it=0; it<helper_vector.size(); ++it) {
        helper_vector[it]->second->collectVelocity();
    }
}

void SimulationRunner::calculateForce() {
    // the parallelisation is done in the force solver
    this->forceSolver_.calculateForce(geometryHandler_.getGeometry().getGeometryNodes());
}

void SimulationRunner::collide() {
#pragma omp parallel for schedule(dynamic)
    for (unsigned int it = 0; it < geometryHandler_.getPhysicalNodes().size();
         it++) {
        for (unsigned int i = 0;
             i < geometryHandler_.getPhysicalNodes()[0].size();
             i++) {
            nodes::PhysicalNode* node =
                geometryHandler_.getPhysicalNodes()[it][i];
            node->getFluidSolver().collide();
            for (auto j = node->getCDESolvers().begin();
                 j != node->getCDESolvers().end();
                 j++) {
                (*j)->collide();
            }
        }
    }
}

void SimulationRunner::advect() {
#pragma omp parallel for schedule(dynamic)
    for (size_t it = 0; it < this->geometryHandler_.getPhysicalNodes().size();
         it++)
    { // loop x direction of the grid
        for (unsigned int i = 0;
             i < this->geometryHandler_.getPhysicalNodes()[0].size();
             i++)
        { // loop y direction of the grid
            nodes::PhysicalNode* node =
                this->geometryHandler_.getPhysicalNodes()[it][i];
            node->getFluidSolver().advect();
            for (auto j = node->getCDESolvers().begin();
                 j != node->getCDESolvers().end();
                 j++)
            { // loop solvers
                (*j)->advect();
            }
        }
    }
    for (const auto& it : this->geometryHandler_.getBoundaryNodes()) {
        for (const auto bd : it->getBoundarySolvers()) {
            bd.second->advect();
        }
    }
}

void SimulationRunner::initSolvers() {

    for (size_t it = 0; it < geometryHandler_.getPhysicalNodes().size();
         it++) {

        for (const auto& i : geometryHandler_.getPhysicalNodes()[it]) {

            i->getFluidSolver().initSolver();
            for (const auto& cde : i->getCDESolvers()) {
                cde->initSolver();
            }

        }

    }
    reportHandler_.writeReport(0);
}

void SimulationRunner::initSolvers(Field<double> fluidVelocity) {
    for (size_t it = 0; it < geometryHandler_.getPhysicalNodes().size();
         it++) {
        for (const auto& i : geometryHandler_.getPhysicalNodes()[it]) {
            i->getFluidSolver().setVelocity(fluidVelocity);
            i->getFluidSolver().initSolver();
            for (const auto& cde : i->getCDESolvers()) {
                cde->initSolver();
            }
        }
    }
    //    reportHandler_.writeReport(0);
}

void SimulationRunner::preAdvect() {
    typedef std::unordered_set<nodes::BoundaryNode*>::const_iterator ListIterator;
    std::vector<ListIterator> helper_vector;

    // create helper vector for parallelization:
    std::unordered_set<nodes::BoundaryNode*> tempBoundaryNodes = this->geometryHandler_.getBoundaryNodes();
    for (ListIterator it=tempBoundaryNodes.begin();
         it != tempBoundaryNodes.end();
         ++it) {
        helper_vector.push_back(it);
    }

#pragma omp parallel for schedule(dynamic)
    for (size_t it=0;
         it<helper_vector.size();
         ++it) {
        for (const auto bd : (*helper_vector[it])->getBoundarySolvers()) {
            bd.second->preAdvect();
        }
    }
}

void SimulationRunner::postAdvect() {
    typedef std::unordered_set<nodes::BoundaryNode*>::const_iterator ListIterator;
    std::vector<ListIterator> helper_vector;

    // create helper vector for parallelization:
    std::unordered_set<nodes::BoundaryNode*> tempBoundaryNodes = this->geometryHandler_.getBoundaryNodes();
    for (ListIterator it=tempBoundaryNodes.begin();
         it != tempBoundaryNodes.end();
         ++it) {
        helper_vector.push_back(it);
    }

#pragma omp parallel for schedule(dynamic)
    for (size_t it=0;
         it<helper_vector.size();
         ++it) {
        for (const auto bd : (*helper_vector[it])->getBoundarySolvers()) {
            bd.second->postAdvect();
        }
    }
}

void SimulationRunner::applyMassSolvers()
{
    for (auto it : this->massSolvers_) {
        it->calculateMass(this->geometryHandler_.getPhysicalNodes());
    }
}

void SimulationRunner::applyBioSolvers()
{
    for (auto it : this->bioSolvers_) {
        (*it).applyBioProcess(const_cast<geometry::GeometryHandler&>(this->geometryHandler_),
                              this->forceSolver_
                              );
        assert(this->geometryHandler_.checkGeometryIntegrity());
    }
}

void SimulationRunner::addCDESolver(const std::string& name) {
    for (size_t it = 0; it < geometryHandler_.getPhysicalNodes().size();
         it++) {
        for (const auto& i : geometryHandler_.getPhysicalNodes()[it]) {
            i->addCDESolver(name);
        }
    }
}

void SimulationRunner::initForceSolver(const std::string& filename) {
    this->forceSolver_.loadForceFile(filename);
}

const solver::ForceSolver &SimulationRunner::getForceSolver()
{
    return this->forceSolver_;
}

void SimulationRunner::addMassSolver(const std::string& name) {
    this->massSolvers_.push_back(solver::MassSolverFactory::instance().createObject(name));
}

void SimulationRunner::addBioSolver(const std::string &name)
{
    this->bioSolvers_.push_back(solver::BioSolverFactory::instance().createObject(name));
}

void SimulationRunner::setTau(
        double tau,
        const std::string& name) {
    for (size_t it = 0; it < geometryHandler_.getPhysicalNodes().size();
         it++) {
        for (const auto& i : geometryHandler_.getPhysicalNodes()[it]) {
            if (name.empty()) {
                i->getFluidSolver().setTau(tau);
            } else {
                i->getCDESolverSlow(name).setTau(tau);
            }
        }
    }
}

void SimulationRunner::distributeForce() {
    typedef std::map<unsigned int,std::shared_ptr<nodes::GeometryNode> >::const_iterator MapIterator;

    // first reset the forces:
#pragma omp parallel for collapse(1) schedule(dynamic)
    for (size_t it = 0; it < geometryHandler_.getPhysicalNodes().size();
         it++) {
        for (size_t i = 0; i < geometryHandler_.getPhysicalNodes()[0].size();
             i++ )
        {
            geometryHandler_.getPhysicalNodes()[it][i]->getFluidSolver().resetForce();
        }
    }

    // build helper vector with iterators for parallelization:
    std::vector<MapIterator> helper_vector;
    for (MapIterator it = this->geometryHandler_.getGeometry().getGeometryNodes().begin();
         it != this->geometryHandler_.getGeometry().getGeometryNodes().end();
         it++) {
        helper_vector.push_back(it);

    }

    // distribute forces:
#pragma omp parallel for schedule(static)
    for (size_t it=0; it< helper_vector.size(); ++it) {
        helper_vector[it]->second->distributeForce();
    }
}
}  // end namespace
