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
#include <LbmLib/include/solver/CDESolver/DiracD2Q4.hpp>
#include <LbmLib/include/Constants.hpp>

#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Log.hpp>
#include <algorithm>
#include <cassert>
#include <numeric>
#include <string>
#include <random>
namespace LbmLib {
namespace solver {
namespace {
const double pos = 32;  ///< The position
constexpr double D = 0.04;  ///< The diffusion constant
/**
  * @brief calculates the delta dirac at time deltaX
  * @param deltaX The distance where the delta dirac is calculated
  * @return the delta diract impuls
  */
double deltaDiracStart(double deltaX) {
    constexpr double t = 50;
    return std::exp(-deltaX * deltaX / (4.0 * D * t)) / (4.0 * PI * D * t);
}
}

void DiracD2Q4::initSolver() {
    double dist =
        std::sqrt((pos -
                   physicalNode_->getXPos()) *
                (pos -
             physicalNode_->getXPos()) +
                (pos -
             physicalNode_->getYPos()) * (pos - physicalNode_->getYPos()));
    for (auto d : cdeDirIter_) {
        distributions_[d] = deltaDiracStart(dist) * 0.25;
    }

    collide();
}

void DiracD2Q4::writeSolver(std::ostream* const stream) {
    (*stream) << physicalNode_->getXPos() << '\t' << physicalNode_->getYPos();
    for (auto d : distributions_) {
        (*stream) << '\t' << d;
    }
    (*stream) << '\n';
}

void DiracD2Q4::loadSolver(std::stringstream* const stream) {
    int x, y;
    (*stream) >> x >> y;
    assert(physicalNode_->getXPos() == x && "The position does not match");
    assert(physicalNode_->getYPos() == y && "The position does not match");
    for (auto d : cdeDirIter_) {
        (*stream) >> distributions_[d];
    }
}

double& DiracD2Q4::accessDistribution(const Direction& dir) {
    assert(dir > T && dir < NE);
    return distributions_[dir];
}

void DiracD2Q4::rescaleDistributions(const double factor) {
    for (auto &it: this->distributions_) {
        it *= factor;
    }
}

double DiracD2Q4::getC() const {
    return std::accumulate(distributions_.begin(), distributions_.end(), 0.0);
}

void DiracD2Q4::collide() {
    assert(physicalNode_ != nullptr);
    assert(distributions_[0] == 0.0);
    // Calculate the rho
    const double C = getC();

    // calculate the speeds
    const double u = physicalNode_->getFluidSolver().getVelocity().x;
    const double v = physicalNode_->getFluidSolver().getVelocity().y;

    const double w = C * 0.25;

    double temp[5];
    temp[E] = w * (1.0 + 2.0 * u);
    temp[N] = w * (1.0 + 2.0 * v);
    temp[W] = w * (1.0 + 2.0 * (-u));
    temp[S] = w * (1.0 + 2.0 * (-v));

    const double tauI = 1.0 / getTau();

    for (auto d : cdeDirIter_) {
        double tempD = distributions_[d];
        // compute non equilibirum
        // make relaxation
        distributions_[d] = tempD - tempD * tauI + temp[d] * tauI;
    }


    // preparation for advect step
    localSwap();
}

double DiracD2Q4::calculateEquilibrium(const Direction& dir) {
    const double C = getC();
    // calculate the speeds
    const double u = physicalNode_->getFluidSolver().getVelocity().x;
    const double v = physicalNode_->getFluidSolver().getVelocity().y;

    const double w = C / 4.0;

    switch (dir) {
      case E:
          return w * (1.0 + 2.0 * u);
          break;
      case N:
          return w * (1.0 + 2.0 * v);
          break;
      case W:
          return w * (1.0 + 2.0 * (-u));
          break;
      case S:
          return w * (1.0 + 2.0 * (-v));
          break;
      default:
          assert(
                false &&
                "you want to get a inverse direction of Direction that does not exist");
    }
    return 0;
}

void DiracD2Q4::advect() {
    assert(physicalNode_ != nullptr);
    if (physicalNode_->getBoundaryNeighbour(W) == nullptr) {
        std::swap(distributions_[getInverseDirection(
                                         W)],
                physicalNode_->getPhysicalNeighbour(W)->getCDESolver(
                        solverID_).accessDistribution(W));
    }
    if (physicalNode_->getBoundaryNeighbour(S) == nullptr) {
        std::swap(distributions_[getInverseDirection(
                                         S)],
                physicalNode_->getPhysicalNeighbour(S)->getCDESolver(
                        solverID_).accessDistribution(S));
    }
}

void DiracD2Q4::localSwap() {
    std::swap(distributions_[E], distributions_[W]);
    std::swap(distributions_[N], distributions_[S]);
}

void DiracD2Q4::reinitialise() {
    double sumC = 0.0;
    int counter = 0;
    for (auto d : cdeDirIter_) {
        // if it has no boundary neighbour and the neighbour is in the same domain then get the concentration
        if ((this->physicalNode_->getBoundaryNeighbour(d) == nullptr) &&
            (this->physicalNode_->getDomainIdentifier() ==
             this->physicalNode_->getPhysicalNeighbour(d)->
               getDomainIdentifier()) ) {
            sumC += this->physicalNode_->getPhysicalNeighbour(d)->getCDESolver(
                        solverID_).getC();
            counter++;
        }
    }
    if (counter == 0) {
        std::array<Direction, 4> dir {{NE, NW, SW, SE}
        };
        for (auto d : dir) {
            // we need to check the diagonals as it does not work in the other directions
            if (this->physicalNode_->getDomainIdentifier() ==
                this->physicalNode_->getPhysicalNeighbour(d)->
                  getDomainIdentifier()) {
                sumC +=
                    this->physicalNode_->getPhysicalNeighbour(d)->getCDESolver(
                            solverID_).getC();
                counter++;
            }
        }
        LOG(UtilLib::logINFO) <<
            "the default initialisation failed. Therefore the node was reinitialised from the diagonal directions";
    }
    if (counter == 0) {
        UtilLib::Exception(
                "The cde solver failed to reinitialise the node, this might be due to a stange geometry");
    }
    sumC /= static_cast<double>(counter);
    for (auto d : cdeDirIter_) {
        distributions_[d] = sumC / 4.0;
    }
    this->collide();
}

const std::string DiracD2Q4::name = "DiracD2Q4";


CDEDirectionsIteratorD2Q4 const DiracD2Q4::cdeDirIter_ =
    CDEDirectionsIteratorD2Q4();

DiracD2Q4::DiracD2Q4() : BaseCDESolver(),
                         distributions_(std::array<double,
                                         5> {
                                     {0.0, 0.0, 0.0,
                                      0.0, 0.0}
                                 }

                                 )
{}
}
}  // end namespace
