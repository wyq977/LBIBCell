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
#include <LbmLib/include/solver/BioSolver/BioSolverMembraneTension.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/solver/ForceSolver.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <omp.h>

namespace LbmLib {
namespace solver {
namespace {
    const unsigned int FREQUENCY = 10; ///< The frequency of applying this solver
    const double FORCE = 0.02; ///< the surface tension force
    class Connection;
}

BioSolverMembraneTension::BioSolverMembraneTension() : BioBaseSolver()
{}

void BioSolverMembraneTension::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    std::stringstream forcedescriptor;
    auto tempconnectionmap = geometryhandler.getGeometry().getConnections();

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }
    // step 1: clear all forces:
    forcesolver.deleteForceType(6);

    // step 2: renew all tensions:
#pragma omp parallel for schedule(static) private(forcedescriptor)
    for (size_t j=0;
         j<tempconnectionmap.size();
         ++j) {
        forcedescriptor.str( std::string() ); // reset
        forcedescriptor.clear(); //clear flags
        forcedescriptor << "6\t" // type
                        << tempconnectionmap[j]->getGeometryNodes().first->getId() << "\t" // nodeID 1
                        << tempconnectionmap[j]->getGeometryNodes().second->getId() << "\t" // nodeID 2
                        << FORCE;
        forcesolver.addForce(&forcedescriptor);

        forcedescriptor.str( std::string() ); // reset
        forcedescriptor.clear(); //clear flags
        forcedescriptor << "6\t" // type
                        << tempconnectionmap[j]->getGeometryNodes().second->getId() << "\t" // nodeID 2
                        << tempconnectionmap[j]->getGeometryNodes().first->getId() << "\t" // nodeID 1
                        << FORCE;
        forcesolver.addForce(&forcedescriptor);
    }
}

const std::string BioSolverMembraneTension::name = "BioSolverMembraneTension";
}
}  // end namespace

