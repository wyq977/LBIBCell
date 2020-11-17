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
#include <LbmLib/include/solver/MassSolver/MassSolverNoFluxYCDED2Q5.hpp>
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

// no flux 
// 1. change the C of CDE at x=0 to x=1 and x=L to x=1
// 2. change the dir of CDE:
//     * node(x=0).accessDistribution(dir) = calculateEquilibrium(dir) - calculateEquilibrium(dir) + node(x=1).accessDistribution(dir)


MassSolverNoFluxYCDED2Q5::MassSolverNoFluxYCDED2Q5() : BaseMassSolver()
{}

// unsigned int y = 0; y < Parameters.getSizeY()
// unsigned int y = 0; x < Parameters.getSizeX()
// physicalGrid_[y][x]
void MassSolverNoFluxYCDED2Q5::calculateMass(
        const std::vector<std::vector<nodes::PhysicalNode*> >& fluidGrid) {
    
    // get domain size in Y
    unsigned int L = fluidGrid.size();

    // no flux at y=0:
#pragma omp parallel for schedule(dynamic)
    for (size_t itx = 0; itx <fluidGrid[0].size(); itx++) {
        // sink mass at y=0
        scalingfactor = 1.0/fluidGrid[0][itx]->getFluidSolver().getRho();
        fluidGrid[0][itx]->getFluidSolver().rescaleDistributions(scalingfactor);
        // get fluid velocity
        const double ux = fluidGrid[0][itx]->getFluidSolver().getVelocity().x;
        const double uy = fluidGrid[0][itx]->getFluidSolver().getVelocity().y;

        // look at BoundarySolverNoFluxD2Q5        
        for (auto cdes = fluidGrid[0][itx]->getCDESolvers().begin();
             cdes != fluidGrid[0][itx]->getCDESolvers().end();
             cdes++) {
                // neighbour: y=1
                auto nbcdes = fluidGrid[1][itx]->getCDESolvers().begin();
                double Cf = (*nbcdes)->getC() / 6.0;

                // get the velocity on fluidGrid
                double feq = 0.0;
                double fneq = 0.0;
                double newDistribution = 0.0;

                // E
                feq = Cf * (1 + 3 * ux);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(E)) -
                    (*nbcdes)->calculateEquilibrium(E);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(E, newDistribution);
                // N
                feq = Cf * (1 + 3 * uy);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(N)) -
                    (*nbcdes)->calculateEquilibrium(N);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(N, newDistribution);
                // W
                feq = Cf * (1 - 3 * ux);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(W)) -
                    (*nbcdes)->calculateEquilibrium(W);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(W, newDistribution);
                // S
                feq = Cf * (1 - 3 * uy);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(S)) -
                    (*nbcdes)->calculateEquilibrium(S);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(S, newDistribution);
            }
        }

    // no flux at y=L:
#pragma omp parallel for schedule(dynamic)
    for (size_t itx = 0; itx <fluidGrid[0].size(); itx++) {
        // sink mass at y=L
        scalingfactor = 1.0/fluidGrid[L - 1][itx]->getFluidSolver().getRho();
        fluidGrid[L - 1][itx]->getFluidSolver().rescaleDistributions(scalingfactor);
        // get fluid velocity
        const double ux = fluidGrid[L - 1][itx]->getFluidSolver().getVelocity().x;
        const double uy = fluidGrid[L - 1][itx]->getFluidSolver().getVelocity().y;

        // look at BoundarySolverNoFluxD2Q5        
        for (auto cdes = fluidGrid[0][itx]->getCDESolvers().begin();
             cdes != fluidGrid[0][itx]->getCDESolvers().end();
             cdes++) {
                // neighbour: y=L-1
                auto nbcdes = fluidGrid[L - 2][itx]->getCDESolvers().begin();
                double Cf = (*nbcdes)->getC() / 6.0;

                // get the velocity on fluidGrid
                double feq = 0.0;
                double fneq = 0.0;
                double newDistribution = 0.0;

                // E
                feq = Cf * (1 + 3 * ux);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(E)) -
                    (*nbcdes)->calculateEquilibrium(E);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(E, newDistribution);
                // N
                feq = Cf * (1 + 3 * uy);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(N)) -
                    (*nbcdes)->calculateEquilibrium(N);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(N, newDistribution);
                // W
                feq = Cf * (1 - 3 * ux);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(W)) -
                    (*nbcdes)->calculateEquilibrium(W);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(W, newDistribution);
                // S
                feq = Cf * (1 - 3 * uy);
                fneq =
                    (*nbcdes)->accessDistribution(getInverseDirection(S)) -
                    (*nbcdes)->calculateEquilibrium(S);
                newDistribution = feq + fneq;
                (*nbcdes)->setDistribution(S, newDistribution);
            }
        }

}

const std::string MassSolverNoFluxYCDED2Q5::name = "MassSolverNoFluxYCDED2Q5";
}
}  // end namespace

