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
#include <LbmLib/include/solver/BioSolver/BioSolverGrowth.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <vector>
#include <string>
#include <iostream>
namespace LbmLib {
namespace solver {

namespace {
    const unsigned int FREQUENCY = 1; ///< The frequency of applying this solver
    const double source = 0.0025; ///< the mass increment which is added at each time step
    double cdeconcentration; ///< the concentration of the cde solver
}

BioSolverGrowth::BioSolverGrowth() : BioBaseSolver()
{}

void BioSolverGrowth::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // add mass to the cells:
#pragma omp parallel for schedule(dynamic)
    for (unsigned int ity = 0;
         ity < geometryhandler.getPhysicalNodes().size();
         ity++) { // loop y direction
        for (unsigned int itx = 0;
             itx < geometryhandler.getPhysicalNodes()[0].size();
             itx++) { // loop x direction
            if (geometryhandler.getPhysicalNodes()[ity][itx]->getDomainIdentifier() != 0 ) {
                //cdeconcentration = fluidGrid[it][i]->getCDESolverSlow("CDESolverD2Q5HH").getC();
                geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(source);
            }
        }
    }
}

const std::string BioSolverGrowth::name = "BioSolverGrowth";
}
}  // end namespace

