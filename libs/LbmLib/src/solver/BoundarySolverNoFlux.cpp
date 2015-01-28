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
#include <LbmLib/include/solver/BoundarySolverNoFlux.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>

#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <utility>
#include <algorithm>
#include <string>

namespace LbmLib {
namespace solver {
BoundarySolverNoFlux::BoundarySolverNoFlux() : BaseBoundarySolver(),
                                               distribution_(0.0)
{}

void BoundarySolverNoFlux::postAdvect() {
    for (const auto& s : cdeSolvers_) {
        const Direction dir = boundaryNode_->getDirectionToNeighbour();
        if ((dir > T) && (dir < NE)) {
            std::swap(distribution_,
                    boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(
                            s).accessDistribution(boundaryNode_->
                              getDirectionToNeighbour()));
        }
    }
}

void BoundarySolverNoFlux::advect() {}

void BoundarySolverNoFlux::preAdvect() {
    for (const auto& s : cdeSolvers_) {
        const double Cf =
            boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).getC() /
            4.0;
        const double ux = boundaryNode_->getXVelocity();
        const double uy = boundaryNode_->getYVelocity();
        const Direction dir = boundaryNode_->getDirectionToNeighbour();

        double feq = 0.0;
        double fneq = 0.0;
        switch (dir) {
          case E:
              feq = Cf * (1 + 2 * ux);
              fneq =
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    accessDistribution(
                          getInverseDirection(dir)) -
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    calculateEquilibrium(dir);
              distribution_ = feq + fneq;
              return;
              break;
          case N:
              feq = Cf * (1 + 2 * uy);
              fneq =
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    accessDistribution(
                          getInverseDirection(dir)) -
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    calculateEquilibrium(dir);
              distribution_ = feq + fneq;
              return;
              break;
          case W:
              feq = Cf * (1 - 2 * ux);
              fneq =
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    accessDistribution(
                          getInverseDirection(dir)) -
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    calculateEquilibrium(dir);
              distribution_ = feq + fneq;
              return;
              break;
          case S:
              feq = Cf * (1 - 2 * uy);
              fneq =
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    accessDistribution(
                          getInverseDirection(dir)) -
                  boundaryNode_->getPhysicalNeighbour()->getCDESolverSlow(s).
                    calculateEquilibrium(dir);
              distribution_ = feq + fneq;
              return;
              break;
          default:
              UtilLib::Exception(
                    "for the boundary condition only e,n,w and s direction exists there must somewhere be an error");
              break;
        }
    }
}

void BoundarySolverNoFlux::initSolver() {}

double& BoundarySolverNoFlux::accessDistribution(const Direction& dir) {
    return distribution_;
}

const std::string BoundarySolverNoFlux::name = "BoundarySolverNoFlux";
}
}  // end namespace
