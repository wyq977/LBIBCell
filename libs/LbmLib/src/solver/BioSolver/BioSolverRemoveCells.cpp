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
#include <LbmLib/include/solver/BioSolver/BioSolverRemoveCells.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <omp.h>

namespace LbmLib {
namespace solver {
namespace {
    const unsigned int FREQUENCY = 10; ///< The frequency of applying this solver.

    const double XMIN = 25.0;
    const double XMAX = 975.0;
    const double YMIN = 25.0;
    const double YMAX = 975.;

    const double MINIMALAREA = 50.0;
}

BioSolverRemoveCells::BioSolverRemoveCells() : BioBaseSolver()
{}

void BioSolverRemoveCells::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {

    // skip if not every FREQUENCY iteration
    if (Parameters.getCurrentIteration() % FREQUENCY != 0) {
        return;
    }

    // get the id of cells to remove
    std::set<unsigned int> cellsToDelete;
    auto tempnodesmap = geometryhandler.getGeometry().getGeometryNodes();

    // remove cells too close to bound
    for (auto i : tempnodesmap) {
        if (i.second->getXPos() < XMIN || i.second->getXPos() > XMAX || i.second->getYPos() < YMIN || i.second->getYPos() > YMAX) {
            cellsToDelete.insert(i.second->getDomainIdOfAdjacentConnections());
        }
    }

    // also remove too small cells: area limit: MINIMALAREA
    std::map<unsigned int,double> areas;
    areas = geometryhandler.computeAreas();
    for (auto it : areas) {
        if (it.second < MINIMALAREA) {
            cellsToDelete.insert(it.first);
        }
    }

    // remove the cells
    for (auto it : cellsToDelete) {
        geometryhandler.removeCell(forcesolver,it);
    }

}

const std::string BioSolverRemoveCells::name = "BioSolverRemoveCells";
}
}  // end namespace