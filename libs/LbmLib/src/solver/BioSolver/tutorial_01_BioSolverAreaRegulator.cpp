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
#include <LbmLib/include/solver/BioSolver/tutorial_01_BioSolverAreaRegulator.hpp>
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
    const unsigned int FREQUENCY = 1; ///< The frequency of applying this solver.
    const double TARGETAREA = 200.0; ///< The default target cell area.
    const double KP = 0.00001; ///< The proportional constant.
    const double MAXSOURCE = 0.004; ///< The threshold for the maximal source. 
}

tutorial_01_BioSolverAreaRegulator::tutorial_01_BioSolverAreaRegulator() : BioBaseSolver()
{}

void tutorial_01_BioSolverAreaRegulator::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    std::map<unsigned int,double> areas;
    std::map<unsigned int,double> sourcemap;
    unsigned int domainid;

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // compute areas
    areas = geometryhandler.computeAreas();

    // update targetareamap
    for (auto it : areas) {
        if ( this->targetareamap_.find(it.first) == this->targetareamap_.end() ) { // not existing yet
            this->targetareamap_[it.first] = TARGETAREA; // default
        }
    }

    // compute source
    for (auto it : areas) {
        sourcemap[it.first] = KP*(this->targetareamap_[it.first] - areas[it.first]);
    }

    // modify the sourcemap
    for (auto it : sourcemap)
    {
        if (sourcemap[it.first] > MAXSOURCE) { // upper threshold
            sourcemap[it.first] = MAXSOURCE;
        }
        else if (sourcemap[it.first] < -MAXSOURCE) {
            sourcemap[it.first] = -MAXSOURCE; // lower threshold
        }
    }

    // add mass to the cells
#pragma omp parallel for schedule(dynamic) private(domainid)
    for (unsigned int ity = 0;
         ity < geometryhandler.getPhysicalNodes().size();
         ity++) { // loop y direction
        for (unsigned int itx = 0;
             itx < geometryhandler.getPhysicalNodes()[0].size();
             itx++) { // loop x direction
            domainid = geometryhandler.getPhysicalNodes()[ity][itx]->getDomainIdentifier();
            if (domainid != 0) {
                if (geometryhandler.getPhysicalNodes()[ity][itx]->getCellType()==1) { // max prolif for celltype=1
                    geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(MAXSOURCE);
                }
                else { // regulated area for other celltypes
                    geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(sourcemap[domainid]);
                }
            }
        }
    }
}

const std::string tutorial_01_BioSolverAreaRegulator::name = "tutorial_01_BioSolverAreaRegulator";
}
}  // end namespace
