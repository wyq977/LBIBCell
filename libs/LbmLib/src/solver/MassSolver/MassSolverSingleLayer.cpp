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
#include <LbmLib/include/solver/MassSolver/MassSolverSingleLayer.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>
#include <UtilLib/include/Log.hpp>
#include <vector>
#include <string>
#include <omp.h>

namespace LbmLib {
namespace solver {
MassSolverSingleLayer::MassSolverSingleLayer() : BaseMassSolver()
{}

namespace {
const double massAddition = 0.0001;  ///< the mass added
}

void MassSolverSingleLayer::calculateMass(
        const std::vector<std::vector<nodes::PhysicalNode*> >& fluidGrid) {
#pragma omp parallel for schedule(dynamic)
    for (unsigned int it = 0; it < fluidGrid.size(); it++) {
        for (unsigned int i = 0; i < fluidGrid[0].size(); i++) {
            if (fluidGrid[it][i]->getDomainIdentifier() != 0) {
                fluidGrid[it][i]->getFluidSolver().addMass(massAddition);
            }
        }
    }
}

const std::string MassSolverSingleLayer::name = "MassSolverSingleLayer";
}
}  // end namespace
