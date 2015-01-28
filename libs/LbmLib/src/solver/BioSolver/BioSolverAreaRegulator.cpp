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
#include <LbmLib/include/solver/BioSolver/BioSolverAreaRegulator.hpp>
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
    const unsigned int FREQUENCY = 1; ///< The frequency of applying this solver
    const double TARGETAREA = 200.0; ///< The default target cell area
    const double TARGETAREA2 = 1000; ///< The target cell area for growing cells
    const double KP = 0.00001; ///< The proportional constant
    const double KI = 0.0; ///< The integrative constant.
    const double MAXSOURCE = 0.004; ///< The threshold for the maximal source.
}

BioSolverAreaRegulator::BioSolverAreaRegulator() : BioBaseSolver()
{}

void BioSolverAreaRegulator::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    std::map<unsigned int,double> areas;
    std::map<unsigned int,double> temperror;
    std::map<unsigned int,double> sourcemap;
    std::map<unsigned int,double> uncutsourcemap;
    unsigned int domainid;
    double source;

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // step 0: initialize
    if (Parameters.getCurrentIteration()<5000) { // switch off after 5000
        this->targetareamap_[1] = TARGETAREA2; // only cell 1 shall grow and divide
        this->integratormap_[1] = 0.0; // initialize integrator with 0
    }
    else {
        this->targetareamap_[1] = TARGETAREA; // only cell 1 shall grow and divide
    }

    // step 1: compute areas
    areas = geometryhandler.computeAreas();
    //areas.erase(0); // remove fluid

    // step 2: update targetareamap
    for (auto it : areas) {
        if ( this->targetareamap_.find(it.first) == this->targetareamap_.end() ) { // not existing yet
            this->targetareamap_[it.first] = TARGETAREA; // default
            this->integratormap_[it.first] = 0.0; // initialize integrator with 0
        }
    }

    // step 3: update error map
    for (auto it : areas) {
        temperror[it.first] = this->targetareamap_[it.first] - areas[it.first];
    }

    // step 4: update integral
    for (auto it : areas) {
        this->integratormap_[it.first] += temperror[it.first];
    }

    // step 5: compute source
    for (auto it : areas) {
        uncutsourcemap[it.first] = KP*temperror[it.first] + KI*this->integratormap_[it.first];
    }

    // step 6: cut the sources
    for (auto it : uncutsourcemap)
    {
        if (uncutsourcemap[it.first] > MAXSOURCE) {
            sourcemap[it.first] = MAXSOURCE;
        }
        else if (uncutsourcemap[it.first] < 0.0) {
            sourcemap[it.first] = 0.0; // no negative source
        }
        else {
            sourcemap[it.first] = uncutsourcemap[it.first];
        }
    }

    // step 5: add mass to the cells:
#pragma omp parallel for schedule(dynamic) private(domainid,source)
    for (unsigned int ity = 0;
         ity < geometryhandler.getPhysicalNodes().size();
         ity++) { // loop y direction
        for (unsigned int itx = 0;
             itx < geometryhandler.getPhysicalNodes()[0].size();
             itx++) { // loop x direction
            domainid = geometryhandler.getPhysicalNodes()[ity][itx]->getDomainIdentifier();
            if (domainid != 0) {
                if (geometryhandler.getPhysicalNodes()[ity][itx]->getCellType()==1) {
                    geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(MAXSOURCE);
                }
                else {
                    source = sourcemap[domainid];
                    geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(source);
                }
            }
        }
    }
}

const std::string BioSolverAreaRegulator::name = "BioSolverAreaRegulator";
}
}  // end namespace
