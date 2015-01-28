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
#include <LbmLib/include/solver/MassSolver/MassSolverBoxOutlet.hpp>
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
double scalingfactor; ///< scaling factor for rescaling outflow boundary condition
}

MassSolverBoxOutlet::MassSolverBoxOutlet() : BaseMassSolver()
{}

void MassSolverBoxOutlet::calculateMass(
        const std::vector<std::vector<nodes::PhysicalNode*> >& fluidGrid) {

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

const std::string MassSolverBoxOutlet::name = "MassSolverBoxOutlet";
}
}  // end namespace
