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
#include <LbmLib/include/solver/CDESolver/SchnakenbergD2Q4u.hpp>

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
const double gamma = 300.0;  ///< the gamma parameter
const double a = 0.3;  ///< the a parameter
double deltaT;  ///< the time step
}

void SchnakenbergD2Q4u::initSolver() {
    // init with C = 1 + eps where eps is a random number with mean 0 and std 0.01
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-0.01, 0.01);
    double temp = 0.25 * (1.0 + dis(gen));
    for (auto d : cdeDirIter_) {
        distributions_[d] = temp;
    }
    deltaT = 20.0 / Parameters.getIterations();
}

void SchnakenbergD2Q4u::writeSolver(std::ostream* const stream) {
    (*stream) << physicalNode_->getXPos() << '\t' << physicalNode_->getYPos();
    for (auto d : distributions_) {
        (*stream) << '\t' << d;
    }
    (*stream) << '\n';
}

void SchnakenbergD2Q4u::loadSolver(std::stringstream* const stream) {
    int x, y;
    (*stream) >> x >> y;
    assert(physicalNode_->getXPos() == x && "The position does not match");
    assert(physicalNode_->getYPos() == y && "The position does not match");
    for (auto d : cdeDirIter_) {
        (*stream) >> distributions_[d];
    }
}

double& SchnakenbergD2Q4u::accessDistribution(const Direction& dir) {
    assert(dir > T && dir < NE);
    return distributions_[dir];
}

void SchnakenbergD2Q4u::rescaleDistributions(const double factor) {
    for (auto &it: this->distributions_) {
        it *= factor;
    }
}

void SchnakenbergD2Q4u::setDistribution(const Direction& dir, const double newDistribution) {
    assert(dir > T && dir < NE);
    distributions_[dir] = newDistribution;
}

double SchnakenbergD2Q4u::getC() const {
    return std::accumulate(distributions_.begin(), distributions_.end(), 0.0);
}

void SchnakenbergD2Q4u::collide() {
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
    const std::string schnakenbergD2Q4v = "SchnakenbergD2Q4v";
    const double Cv = physicalNode_->getCDESolverSlow(schnakenbergD2Q4v).getC();
    const double reaktionTerm = 0.25 * deltaT * gamma * (a - C + C * C * Cv);

    for (auto d : cdeDirIter_) {
        double tempD = distributions_[d];
        // compute non equilibirum
        // make relaxation
        distributions_[d] = tempD - tempD * tauI + temp[d] * tauI +
            reaktionTerm;
    }


    // preparation for advect step
    localSwap();
}

double SchnakenbergD2Q4u::calculateEquilibrium(const Direction& dir) {
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

void SchnakenbergD2Q4u::advect() {
    assert(physicalNode_ != nullptr);
    std::swap(distributions_[getInverseDirection(W)],
                physicalNode_->getPhysicalNeighbour(W)->getCDESolver(
                        solverID_).accessDistribution(W));
    std::swap(distributions_[getInverseDirection(S)],
                physicalNode_->getPhysicalNeighbour(S)->getCDESolver(
                        solverID_).accessDistribution(S));
}

void SchnakenbergD2Q4u::localSwap() {
    std::swap(distributions_[E], distributions_[W]);
    std::swap(distributions_[N], distributions_[S]);
}

void SchnakenbergD2Q4u::reinitialise() {
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

const std::string SchnakenbergD2Q4u::name = "SchnakenbergD2Q4u";


CDEDirectionsIteratorD2Q4 const SchnakenbergD2Q4u::cdeDirIter_ =
    CDEDirectionsIteratorD2Q4();

SchnakenbergD2Q4u::SchnakenbergD2Q4u() : BaseCDESolver(),
                                         distributions_(std::array<double,
                                                         5> {{0.0, 0.0, 0.0,
                                                              0.0, 0.0}
                                                 }
                                                 )
{}
}
}  // end namespace
