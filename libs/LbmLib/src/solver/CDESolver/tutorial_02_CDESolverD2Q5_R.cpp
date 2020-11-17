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
#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/solver/BoundaryAbstractSolver.hpp>
#include <LbmLib/include/solver/CDESolver/tutorial_02_CDESolverD2Q5_R.hpp>
#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Log.hpp>
#include <algorithm>
#include <cassert>
#include <numeric>
#include <string>
#include <random>
#include <iomanip>

namespace LbmLib {
namespace solver {
namespace {
const double gamma = 100.0; ///< the gamma parameter
const double a = 0.1; ///< the a parameter
const double deltaT = 1.0e-4; ///< the time step
}

void tutorial_02_CDESolverD2Q5_R::initSolver() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-0.01, 0.01);

    if (this->physicalNode_->getDomainIdentifier() == 0) {
        for (auto d : cdeDirIter_) {
            distributions_[d] = 0.0;
        }
    }
    else {
        for (auto d : cdeDirIter_) {
            distributions_[d] = (1.0 + dis(gen))/5.0;
        }
    }
}

void tutorial_02_CDESolverD2Q5_R::writeSolver(std::ostream* const stream) {
    (*stream) << physicalNode_->getXPos() << '\t' << physicalNode_->getYPos();
    for (auto d : distributions_) {
        (*stream) << '\t' << d;
    }
    (*stream) << '\n';
}

void tutorial_02_CDESolverD2Q5_R::loadSolver(std::stringstream* const stream) {
    int x, y;
    (*stream) >> x >> y;
    assert(physicalNode_->getXPos() == x && "The position does not match");
    assert(physicalNode_->getYPos() == y && "The position does not match");
    for (auto d : cdeDirIter_) {
        (*stream) >> distributions_[d];
    }
}

double& tutorial_02_CDESolverD2Q5_R::accessDistribution(const Direction& dir) {
    assert(dir > T && dir < NE);
    return distributions_[dir];
}

void tutorial_02_CDESolverD2Q5_R::rescaleDistributions(const double factor) {
    for (auto &it: this->distributions_) {
        it *= factor;
    }
}

void tutorial_02_CDESolverD2Q5_R::setDistribution(const Direction& dir, const double newDistribution) {
    assert(dir > T && dir < NE);
    distributions_[dir] = newDistribution;
}

double tutorial_02_CDESolverD2Q5_R::getC() const {
    return std::accumulate(distributions_.begin(), distributions_.end(), 0.0);
}

const double tutorial_02_CDESolverD2Q5_R::reaction() const
{
    const double R = this->getC();
    const double L = physicalNode_->getCDESolverSlow("tutorial_02_CDESolverD2Q5_L").getC();

    if (this->physicalNode_->getDomainIdentifier() != 0) {
        return gamma * (a - R + R * R * L);
    }
    else {
        return 0.0;
    }
}

void tutorial_02_CDESolverD2Q5_R::collide() {
    assert(physicalNode_ != nullptr);
    // get the local concentration:
    const double C = getC();

    // get the local velocity:
    const double ux = physicalNode_->getFluidSolver().getVelocity().x;
    const double uy = physicalNode_->getFluidSolver().getVelocity().y;

    const double w0 = C / 3.0;
    const double w1 = C / 6.0;
    const double tauI = 1.0 / getTau();

    // temporary computations
    double temp[5];
    temp[T] = w0;
    temp[E] = w1 * (1.0 + ux * 3.0);
    temp[N] = w1 * (1.0 + uy * 3.0);
    temp[W] = w1 * (1.0 + (-ux) * 3.0);
    temp[S] = w1 * (1.0 + (-uy) * 3.0);

    const double reactionTerm = this->reaction();
    const double reactionTerm_ZERO = deltaT*reactionTerm / 3.0; // for the ZERO distribution
    const double reactionTerm_OTHERS = deltaT*reactionTerm / 6.0; // for all the other distributions

    distributions_[T] = distributions_[T] - distributions_[T] * tauI + temp[T] * tauI + reactionTerm_ZERO;
    distributions_[E] = distributions_[E] - distributions_[E] * tauI + temp[E] * tauI + reactionTerm_OTHERS;
    distributions_[N] = distributions_[N] - distributions_[N] * tauI + temp[N] * tauI + reactionTerm_OTHERS;
    distributions_[W] = distributions_[W] - distributions_[W] * tauI + temp[W] * tauI + reactionTerm_OTHERS;
    distributions_[S] = distributions_[S] - distributions_[S] * tauI + temp[S] * tauI + reactionTerm_OTHERS;

    // preparation for advect step: this is necessary
    localSwap();
}

double tutorial_02_CDESolverD2Q5_R::calculateEquilibrium(const Direction& dir) {
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

void tutorial_02_CDESolverD2Q5_R::advect() {
    assert(physicalNode_ != nullptr);
    std::swap(distributions_[getInverseDirection(W)],
            physicalNode_->getPhysicalNeighbour(W)->getCDESolver(
                    solverID_).accessDistribution(W));
    std::swap(distributions_[getInverseDirection(S)],
            physicalNode_->getPhysicalNeighbour(S)->getCDESolver(
                    solverID_).accessDistribution(S));
}

void tutorial_02_CDESolverD2Q5_R::localSwap() {
    std::swap(distributions_[E], distributions_[W]);
    std::swap(distributions_[N], distributions_[S]);
}

void tutorial_02_CDESolverD2Q5_R::reinitialise() {
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
    // if not successful, try {NE,NW,SW,SE}:
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

    // if still fails: initialize with neighbors, even if they are in an other domain:
    if (counter == 0) {
        std::stringstream message;
        message << std::setprecision(12)
                << "Initialization on PhysicalNode "
                << "("<< this->physicalNode_->getXPos()<<","<<this->physicalNode_->getYPos() <<")"
                << " failed at time "
                << Parameters.getCurrentIteration()
                << ". Therefore the node was initialized with all neighbors {E,N,W,S}, ignoring their domainID."
                << " CurrentNodeID="<<this->physicalNode_->getDomainIdentifier();
        for (auto d : dir1) {
            // if it has no boundary neighbour and the neighbour is in the same domain then get the concentration
                sumC += this->physicalNode_->getPhysicalNeighbour(d)->getCDESolver(
                            solverID_).getC();
                counter++;
        }
        LOG(UtilLib::logINFO) << message.str().c_str();
    }

    sumC /= static_cast<double>(counter);
    for (auto d : cdeDirIter_) {
        this->distributions_[d] = sumC / 5.0;
    }
    this->collide();
}

const std::string tutorial_02_CDESolverD2Q5_R::name = "tutorial_02_CDESolverD2Q5_R";


CDEDirectionsIteratorD2Q5 const tutorial_02_CDESolverD2Q5_R::cdeDirIter_ =
    CDEDirectionsIteratorD2Q5();

tutorial_02_CDESolverD2Q5_R::tutorial_02_CDESolverD2Q5_R() : BaseCDESolver(),
                                         distributions_(std::array<double,
                                                         5> {
                                                     {0.0, 0.0, 0.0,
                                                      0.0, 0.0}
                                                 }

                                                 )
{}
}
}  // end namespace

