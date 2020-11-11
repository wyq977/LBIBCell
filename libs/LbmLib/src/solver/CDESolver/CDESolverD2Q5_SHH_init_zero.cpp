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
#include <LbmLib/include/solver/CDESolver/CDESolverD2Q5_SHH_init_zero.hpp>
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
const double deltaT = 1.0;  ///< the time step
const unsigned int SWITCHOFF_TIME = 5000; ///< the time when the production of SIGNAL is switched off
const double SIGNAL_decay = 0.001; ///< the decay rate of SIGNAL
const double TURNOVER = 0.005; ///< Turn over rate of SHH
const double SIGNAL_production = 0.0001; ///< the production rate of SIGNAL in the inital cell
const double SIGNAL_initalcondition = 1.0; ///< the inital concentration of SIGNAL
}

void CDESolverD2Q5_SHH_init_zero::initSolver() {
    for (auto d : cdeDirIter_) {
        distributions_[d] = 0.0;
    }
}

void CDESolverD2Q5_SHH_init_zero::writeSolver(std::ostream* const stream) {
    (*stream) << physicalNode_->getXPos() << '\t' << physicalNode_->getYPos();
    for (auto d : distributions_) {
        (*stream) << '\t' << d;
    }
    (*stream) << '\n';
}

void CDESolverD2Q5_SHH_init_zero::loadSolver(std::stringstream* const stream) {
    int x, y;
    (*stream) >> x >> y;
    assert(physicalNode_->getXPos() == x && "The position does not match");
    assert(physicalNode_->getYPos() == y && "The position does not match");
    for (auto d : cdeDirIter_) {
        (*stream) >> distributions_[d];
    }
}

double& CDESolverD2Q5_SHH_init_zero::accessDistribution(const Direction& dir) {
    assert(dir > T && dir < NE);
    return distributions_[dir];
}

void CDESolverD2Q5_SHH_init_zero::rescaleDistributions(const double factor) {
    for (auto &it: this->distributions_) {
        it *= factor;
    }
}

double CDESolverD2Q5_SHH_init_zero::getC() const {
    return std::accumulate(distributions_.begin(), distributions_.end(), 0.0);
}

const double CDESolverD2Q5_SHH_init_zero::reaction() const
{
    if ((this->physicalNode_->getCellType() == 2) && (Parameters.getCurrentIteration()<SWITCHOFF_TIME)) {
        return SIGNAL_production - TURNOVER * this->getC();
    }
    else if (this->physicalNode_->getCellType() == 1) {
        // exponetial turn-over on the cell 
        // if just: - 0.00002, negative value will be seen, does not make sense
        return - TURNOVER * this->getC();
    }
    else {
        return -SIGNAL_decay*this->getC();
    }
}

void CDESolverD2Q5_SHH_init_zero::collide() {
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

double CDESolverD2Q5_SHH_init_zero::calculateEquilibrium(const Direction& dir) {
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

void CDESolverD2Q5_SHH_init_zero::advect() {
    assert(physicalNode_ != nullptr);
    std::swap(distributions_[getInverseDirection(W)],
            physicalNode_->getPhysicalNeighbour(W)->getCDESolver(
                    solverID_).accessDistribution(W));
    std::swap(distributions_[getInverseDirection(S)],
            physicalNode_->getPhysicalNeighbour(S)->getCDESolver(
                    solverID_).accessDistribution(S));
}

void CDESolverD2Q5_SHH_init_zero::localSwap() {
    std::swap(distributions_[E], distributions_[W]);
    std::swap(distributions_[N], distributions_[S]);
}

void CDESolverD2Q5_SHH_init_zero::reinitialise() {
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

const std::string CDESolverD2Q5_SHH_init_zero::name = "CDESolverD2Q5_SHH_init_zero";


CDEDirectionsIteratorD2Q5 const CDESolverD2Q5_SHH_init_zero::cdeDirIter_ =
    CDEDirectionsIteratorD2Q5();

CDESolverD2Q5_SHH_init_zero::CDESolverD2Q5_SHH_init_zero() : BaseCDESolver(),
                                         distributions_(std::array<double,
                                                         5> {
                                                     {0.0, 0.0, 0.0,
                                                      0.0, 0.0}
                                                 }

                                                 )
{}
}
}  // end namespace

