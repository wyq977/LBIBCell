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
#include <LbmLib/include/solver/MassSolver/MassSolverSingleGrowingCell.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <UtilLib/include/Log.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <omp.h>

namespace LbmLib {
namespace solver {

namespace {
const double source = 0.0025; ///< the mass increment which is added at each time step
double scalingfactor; ///< scaling factor for rescaling outflow boundary condition
double cdeconcentration; ///< the concentration of the cde solver
}


MassSolverSingleGrowingCell::MassSolverSingleGrowingCell() : BaseMassSolver()
{}

void MassSolverSingleGrowingCell::calculateMass(
        const std::vector<std::vector<nodes::PhysicalNode*> >& fluidGrid) {
    // add mass to the cells:
#pragma omp parallel for schedule(dynamic)
    for (unsigned int it = 0; it < fluidGrid.size(); it++) { // loop x direction
        for (unsigned int i = 0; i < fluidGrid[0].size(); i++) { // loop y direction
            //if (fluidGrid[it][i]->getDomainIdentifier() ==
            //        1) { // only bottom cell
            if (Parameters.getCurrentIteration() < 4000) {
            if (fluidGrid[it][i]->getDomainIdentifier() != 0 ) {
                //cdeconcentration = fluidGrid[it][i]->getCDESolverSlow("CDESolverD2Q5HH").getC();
                    fluidGrid[it][i]->getFluidSolver().addMass(source);
            }
            }

//            else if (fluidGrid[it][i]->getCellType() == 2 ) {
//                cdeconcentration = fluidGrid[it][i]->getCDESolverSlow("CDESolverD2Q5HH").getC();
//                    fluidGrid[it][i]->getFluidSolver().addMass(0.2*source);
//            }
        }
    }

    // sink at x=0:
#pragma omp parallel for schedule(dynamic)
    for (size_t ity = 0; ity < fluidGrid.size(); ity++) {
        scalingfactor = 1.0/fluidGrid[ity][0]->getFluidSolver().getRho();
        fluidGrid[ity][0]->getFluidSolver().rescaleDistributions(scalingfactor); // rescale fluid mass

        // when sinking away mass, also sink away CDE species at same rate:
        for (auto cdes = fluidGrid[ity][0]->getCDESolvers().begin();
             cdes != fluidGrid[ity][0]->getCDESolvers().end();
             cdes++) {
            (*cdes)->rescaleDistributions(scalingfactor);
        }
    }

    // sink at y=0:
#pragma omp parallel for schedule(dynamic)
    for (size_t itx = 0; itx <fluidGrid[0].size(); itx++) {
        scalingfactor = 1.0/fluidGrid[0][itx]->getFluidSolver().getRho();
        fluidGrid[0][itx]->getFluidSolver().rescaleDistributions(scalingfactor);

        // when sinking away mass, also sink away CDE species at same rate:
        for (auto cdes = fluidGrid[0][itx]->getCDESolvers().begin();
             cdes != fluidGrid[0][itx]->getCDESolvers().end();
             cdes++) {
            (*cdes)->rescaleDistributions(scalingfactor);
        }
    }
}

const std::string MassSolverSingleGrowingCell::name = "MassSolverSingleGrowingCell";
}
}  // end namespace

