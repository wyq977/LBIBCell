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
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/BoundaryAbstractSolver.hpp>
#include <LbmLib/include/solver/CDESolver/CDESolverD2Q5.hpp>
#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Log.hpp>
#include <algorithm>
#include <cassert>
#include <numeric>
#include <string>
namespace LbmLib {
namespace solver {


void CDESolverD2Q5::initSolver() {
    distributions_[T] = 1.0;
    collide();
}

void CDESolverD2Q5::loadSolver(std::stringstream* const stream) {
    int x, y;
    (*stream) >> x >> y;
    assert(physicalNode_->getXPos() == x && "The position does not match");
    assert(physicalNode_->getYPos() == y && "The position does not match");
    for (auto d : cdeDirIter_) {
        (*stream) >> distributions_[d];
    }
}

void CDESolverD2Q5::writeSolver(std::ostream* const stream) {
    (*stream) << physicalNode_->getXPos() << '\t' << physicalNode_->getYPos();
    for (auto d : distributions_) {
        (*stream) << '\t' << d;
    }
    (*stream) << '\n';
}

double& CDESolverD2Q5::accessDistribution(const Direction& dir) {
    assert(dir > T && dir < NE);
    return distributions_[dir];
}

void CDESolverD2Q5::rescaleDistributions(const double factor) {
    for (auto &it: this->distributions_) {
        it *= factor;
    }
}

void CDESolverD2Q5::setDistribution(const Direction& dir, const double newDistribution) {
    assert(dir > T && dir < NE);
    distributions_[dir] = newDistribution;
}

double CDESolverD2Q5::getC() const {
    return std::accumulate(distributions_.begin(), distributions_.end(), 0.0);
}

void CDESolverD2Q5::collide() {
    assert(physicalNode_ != nullptr);
    // Calculate the rho
    const double C = getC();

    // calculate the speeds
    const double ux = physicalNode_->getFluidSolver().getVelocity().x;
    const double uy = physicalNode_->getFluidSolver().getVelocity().y;
    //    std::cout<<std::sqrt(ux*ux+uy*uy)<<std::endl;
    const double w0 = C / 3.0;
    const double w1 = C / 6.0;
    const double tauI = 1.0 / getTau();


    double temp[5];
    temp[T] = w0;
    temp[E] = w1 * (1.0 + ux * 3.0);
    temp[N] = w1 * (1.0 + uy * 3.0);
    temp[W] = w1 * (1.0 + (-ux) * 3.0);
    temp[S] = w1 * (1.0 + (-uy) * 3.0);


    for (auto d : cdeDirIter_) {
        double tempD = distributions_[d];
        // compute non equilibirum
        // make relaxation
        distributions_[d] = tempD - tempD * tauI + temp[d] * tauI;
    }


    // preparation for advect step
    localSwap();
}

double CDESolverD2Q5::calculateEquilibrium(const Direction& dir) {
    const double C = getC();
    // calculate the speeds
    const double u = physicalNode_->getFluidSolver().getVelocity().x;
    const double v = physicalNode_->getFluidSolver().getVelocity().y;

    const double w1 = C / 6.0;

    switch (dir) {
      case T:
          return C / 3.0;
          break;
      case E:
          return w1 * (1.0 + u * 3.0);
          break;
      case N:
          return w1 * (1.0 + v * 3.0);
          break;
      case W:
          return w1 * (1.0 + (-u) * 3.0);
          break;
      case S:
          return w1 * (1.0 +  (-v) * 3.0);
          break;
      default:
          assert(
                false &&
                "you want to get a inverse direction of a Direction that does not exist");
    }
    return 0;
}

void CDESolverD2Q5::advect() {
    assert(physicalNode_ != nullptr);
    std::swap(distributions_[getInverseDirection(W)],
                physicalNode_->getPhysicalNeighbour(W)->getCDESolver(
                        solverID_).accessDistribution(W));
    std::swap(distributions_[getInverseDirection(S)],
                physicalNode_->getPhysicalNeighbour(S)->getCDESolver(
                        solverID_).accessDistribution(S));
}

void CDESolverD2Q5::localSwap() {
    std::swap(distributions_[E], distributions_[W]);
    std::swap(distributions_[N], distributions_[S]);
}

void CDESolverD2Q5::reinitialise() {
    double sumC = 0.0;
    int counter = 0;
    const unsigned int myNodeID = this->physicalNode_->getDomainIdentifier();

    // accumulate the concenctration of appropriate (tbd) neighbors.
    // first, try {E, N, W, S}:
    std::array<Direction, 4> dir1 {{E, N, W, S}};
    for (auto d : dir1) {
        // if it has no boundary neighbour and the neighbour is in the same domain then get the concentration
        if ((this->physicalNode_->getBoundaryNeighbour(d) == nullptr) &&
            (myNodeID ==
             this->physicalNode_->getPhysicalNeighbour(d)->getDomainIdentifier()) ) {
            sumC += this->physicalNode_->getPhysicalNeighbour(d)->getCDESolver(
                        solverID_).getC();
            counter++;
        }
    }
    // fi not successful, try {NE,NW,SW,SE}:
    if (counter == 0) {
        std::array<Direction, 4> dir2 {{NE, NW, SW, SE}};
        for (auto d : dir2) {
            // we need to check the diagonals as it does not work in the other directions
            if ( (this->physicalNode_->getBoundaryNeighbour(d) == nullptr) &&
                    (myNodeID == this->physicalNode_->getPhysicalNeighbour(d)->getDomainIdentifier())) {
                sumC +=
                    this->physicalNode_->getPhysicalNeighbour(d)->getCDESolver(
                            solverID_).getC();
                counter++;
            }
        }
        if (counter != 0) {
            std::stringstream message;
            message << std::setprecision(12);
            message << "Default initialisation on PhysicalNode ";
            message << "("<< this->physicalNode_->getXPos()<<","<<this->physicalNode_->getYPos() <<")";
            message << " failed. Therefore the node was reinitialised from the diagonal directions";
            LOG(UtilLib::logINFO) << message.str().c_str();
        }

    }
    if (counter == 0) {
        std::stringstream message;
        message << std::setprecision(12);
        message << "Initialization on PhysicalNode ";
        message << "("<< this->physicalNode_->getXPos()<<","<<this->physicalNode_->getYPos() <<")";
        message << " failed at time ";
        message << Parameters.getCurrentIteration();
        message << ". Therefore the node was NOT INITIALIZED.";
        message << " CurrentNodeID="<<this->physicalNode_->getDomainIdentifier();
        message << ". DirectNeighbor(domainID): {";

        std::array<Direction, 8> dir3 {{E, N, W, S, NE, NW, SW, SE}};
        for (auto d : dir3) {
            // if it has no boundary neighbour and the neighbour is in the same domain then get the concentration
            message << d << "(" << this->physicalNode_->getPhysicalNeighbour(d)->getDomainIdentifier() << ")";
            if ( (this->physicalNode_->getBoundaryNeighbour(d) != nullptr)) {
                message <<"<-hasBN";
            }
            message << " ";
        }
        message << "}";


        LOG(UtilLib::logINFO) << message.str().c_str();
        //lbm_fail(message.str().c_str());
    }
    sumC /= static_cast<double>(counter);
    for (auto d : cdeDirIter_) {
        this->distributions_[d] = sumC / 5.0;
    }
    this->collide();
}

CDESolverD2Q5::CDESolverD2Q5() : BaseCDESolver(),
                                 distributions_(std::array<double,
                                                 5> {{0.0, 0.0, 0.0, 0.0, 0.0}
                                         }
                                         )
{}


const std::string CDESolverD2Q5::name = "CDESolverD2Q5";

CDEDirectionsIteratorD2Q5 const CDESolverD2Q5::cdeDirIter_ =
    CDEDirectionsIteratorD2Q5();
}
}  // end namespace
