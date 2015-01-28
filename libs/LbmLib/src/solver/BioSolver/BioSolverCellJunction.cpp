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
#include <LbmLib/include/solver/BioSolver/BioSolverCellJunction.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/solver/ForceSolver.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <random>
#include <memory>
#include <omp.h>

namespace LbmLib {
namespace solver {
namespace {
    const unsigned int FREQUENCY = 10; ///< The frequency of applying this solver
    const int seed = 1; ///< fixed seed for debugging
    static std::mt19937 gen(seed); ///< rng @return
    //static std::random_device rd;
    //static std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1,2); ///< distribution @return
    std::stringstream forcedescriptor; ///< string, as e.g. read from forceIn.txt
    const double RADIUS = 1.0; ///< the cell junction radius
    const double K = 0.02; ///< the cell junction spring constant
    const double L0 = 0.5; ///< the cell junction resting length
}

BioSolverCellJunction::BioSolverCellJunction() : BioBaseSolver()
{}

void BioSolverCellJunction::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // step 1: delete old junctions
    forcesolver.deleteForceType(7);

    // step 2: build new junctions
    auto tempnodesmap = geometryhandler.getGeometry().getGeometryNodes();
    auto nodes_length = tempnodesmap.size();
    std::vector<std::map<unsigned int,std::shared_ptr<nodes::GeometryNode> >::const_iterator> helper_nodes;
    auto nodes_it = tempnodesmap.begin();

    // create array of iterators sequentally:
    for (size_t j=0;
         j<nodes_length;
         ++j) {
        helper_nodes.push_back(nodes_it++);
    }

#pragma omp parallel for schedule(static) private(forcedescriptor)
    for (size_t j=0;
         j<nodes_length;
         ++j) {
        std::shared_ptr<nodes::GeometryNode> closestnode =
                geometryhandler.getGeometry().getGeometryNodesWithinRadiusWithAvoidanceClosest(helper_nodes[j]->second->getXPos(),
                                                                                               helper_nodes[j]->second->getYPos(),
                                                                                               RADIUS,
                                                                                               helper_nodes[j]->second->getDomainIdOfAdjacentConnections());
        if (closestnode != nullptr) {
            forcedescriptor.str( std::string() ); // reset
            forcedescriptor.clear(); //clear flags
            forcedescriptor << "7\t" // type
                            << helper_nodes[j]->second->getId() << "\t" // nodeID 1
                            << closestnode->getId() << "\t" // nodeID 2
                            << K << "\t" // spring constant
                            << L0; // resting length
            forcesolver.addForce(&forcedescriptor);

            forcedescriptor.str( std::string() ); // reset
            forcedescriptor.clear(); //clear flags
            forcedescriptor << "7\t" // type
                            << closestnode->getId() << "\t" // nodeID 1
                            << helper_nodes[j]->second->getId() << "\t" // nodeID 2
                            << K << "\t" // spring constant
                            << L0; // resting length
            forcesolver.addForce(&forcedescriptor);
        }
    }
}

const std::string BioSolverCellJunction::name = "BioSolverCellJunction";
}
}  // end namespace

