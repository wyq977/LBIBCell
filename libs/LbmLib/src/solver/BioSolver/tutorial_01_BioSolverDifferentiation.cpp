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
#include <LbmLib/include/solver/BioSolver/tutorial_01_BioSolverDifferentiation.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <mutex>

namespace LbmLib {
namespace solver {
namespace{
    const unsigned int FREQUENCY = 100; ///< The frequency of applying this solver
    const double THRESHOLD = 1.6; ///< threshold for differentiation policy
    std::once_flag flag; ///< helper flag
}

tutorial_01_BioSolverDifferentiation::tutorial_01_BioSolverDifferentiation() : BioBaseSolver()
{}

void tutorial_01_BioSolverDifferentiation::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // step 1: compute accumulated concentrations per domain [domainID,accumulatedconentration]
    std::map<unsigned int,double> accumulatedcellconcentrationmap =
    geometryhandler.computeAccumulatedDomainConcentrations("tutorial_01_CDESolverD2Q5_SIGNAL");

    // step 2: update cellDefinition_
    this->cellDefinition_.clear();
    for (auto it : geometryhandler.getGeometry().getConnections()) {
        this->cellDefinition_[(*it).getDomainIdentifier()].push_back(it);
    }

    // step 2b: initialize cellTypeTracker_ (only once!)
    std::call_once(flag, [this,&geometryhandler](){
                   geometryhandler.getCellTypeTrackerMap().clear();
                   geometryhandler.getCellTypeTrackerMap()[0] = 0;  // fluid default
                   for (auto it : this->cellDefinition_) { // default all other cells to type 1
                       geometryhandler.getCellTypeTrackerMap()[it.first] = 1;
                   }
    });

    // step 3: differentiate accordingly
    for(std::map<unsigned int,double>::iterator it = ++accumulatedcellconcentrationmap.begin(); //skip extracellular domain
        it != accumulatedcellconcentrationmap.end();
        it++) {
        if (it->second < THRESHOLD) {
            geometryhandler.getCellTypeTrackerMap()[it->first] = 2;
        }
    }

    // step 4: copy cell types to all physical nodes:
    geometryhandler.copyCellTypeToPhysicalNodes(
            geometryhandler.getCellTypeTrackerMap()
            );
}

const std::string tutorial_01_BioSolverDifferentiation::name = "tutorial_01_BioSolverDifferentiation";
}
}  // end namespace

