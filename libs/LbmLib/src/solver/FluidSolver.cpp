/* Copyright (c) 2012 David Sichau <mail"at"sichau"dot"eu>

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
   LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
   NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <LbmLib/include/Constants.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/FluidSolver.hpp>
#include <UtilLib/include/Log.hpp>
#include <algorithm>
#include <cassert>
#include <numeric>
#include <sstream>
#include <string>
namespace LbmLib {
namespace solver {
namespace {
/**
 * @brief W0 The weight in the 0 direction
 */
constexpr double W0 = 4.0 / 9.0;
/**
 * @brief W1 The weight in the E,N,W,S direction
 */
constexpr double W1 = 1.0 / 9.0;
/**
 * @brief W2 the weight in the NE,NW,SW,SE direction
 */
constexpr double W2 = 1.0 / 36.0;
}



void FluidSolver::initSolver() {
    if (velocityInit_) {
        initWithVelocity();
    } else {
        // init the populations with rho = 1 and u=0 and v=0
        // hack to set rho to 1 for the init step

        distributions_[T] = 1.0;
        collide();
    }
}

void FluidSolver::initWithVelocity() {
    const double rho = 1.0;

    // calculate the speeds
    const double u = velocity_.x;
    const double v = velocity_.y;

    const double u2 = u * u;
    const double v2 = v * v;

    const double W1rho = W1 * rho;
    const double W2rho = W2 * rho;
    const double W3 = 9.0 / 2.0;
    const double lastTerm = 3.0 / 2.0 * (u2 + v2);

    double temp[9];

    temp[T] = W0 * rho * (1.0 - lastTerm);
    temp[E] = W1rho * (1.0 + 3.0 * u + W3 * u2 - lastTerm);
    temp[NE] = W2rho * (1.0 + 3.0 * (u + v) + W3 * (u + v) * (u + v) - lastTerm);
    temp[N] = W1rho * (1.0 + 3.0 * v + W3 * v2 - lastTerm);
    temp[NW] = W2rho *
        (1.0 + 3.0 * (-u + v) + W3 * (-u + v) * (-u + v) - lastTerm);
    temp[W] = W1rho * (1.0 + 3.0 * (-u) + W3 * u2 - lastTerm);
    temp[SW] = W2rho *
        (1.0 + 3.0 * (-u - v) + W3 * (-u - v) * (-u - v) - lastTerm);
    temp[S] = W1rho * (1.0 + 3.0 * (-v) + W3 * v2 - lastTerm);
    temp[SE] = W2rho * (1.0 + 3.0 * (u - v) + W3 * (u - v) * (u - v) - lastTerm);

    for (auto d : dirIter_) {
        // assign the equilibirum to the distribution
        // make relaxation
        distributions_[d] = temp[d];
    }


    // reseting the rho to add to zero as it is shifted to the distributions
    rhoToAdd_ = 0.0;
    localSwap();
}

void FluidSolver::collide() {
    //this->rhoToAdd_ = 0.0;

    // Calculate the rho
    double rho = this->getRho();
    assert(rho>0.0);
    if (std::isfinite(this->rhoToAdd_)) {
        rho += this->rhoToAdd_;
    }
    if (rho>0.0) {

    }
    else {
        std::cout<<"rhoToAdd="<<this->rhoToAdd_<<std::endl;
    }
    assert(rho>0.0);
    double rhoI = 1.0 / rho;
    assert(std::isfinite(rhoI));

    // calculate the velocity
    const double u = (distributions_[E] + distributions_[NE]
                      + distributions_[SE]
                      - (distributions_[NW] + distributions_[W]
                         + distributions_[SW])) * rhoI;
    const double u2 = u * u;

    const double v = (distributions_[NE] + distributions_[N]
                      + distributions_[NW]
                      - (distributions_[SW] + distributions_[S]
                         + distributions_[SE])) * rhoI;

    const double v2 = v * v;


    const double lastTerm = 3.0 / 2.0 * (u2 + v2);
    const double tauI = 1.0 / getTau();

    double temp[9];

    const double W3 = 9.0 / 2.0;
    const double W1rho = W1 * rho;
    const double W2rho = W2 * rho;

    // the force calculation is added to the temp calculation
    // an if condition to check if the force is zero is the same speed as simply calculate this
    temp[T] = W0 * rho * (1.0 - lastTerm) * tauI;
    temp[E] = W1rho * (1.0 + 3.0 * u + W3 * u2 - lastTerm) * tauI + W1rho * 3 * force_.x;
    temp[NE] = W2rho * (1.0 + 3.0 * (u + v) + W3 * (u + v) * (u + v) - lastTerm) * tauI + W2rho * 3 * (force_.x + force_.y);
    temp[N] = W1rho * (1.0 + 3.0 * v + W3 * v2 - lastTerm) * tauI + W1rho * 3 * force_.y;
    temp[NW] = W2rho * (1.0 + 3.0 * (-u + v) + W3 * (-u + v) * (-u + v) - lastTerm) * tauI + W2rho * 3 * (-force_.x + force_.y);
    temp[W] = W1rho * (1.0 + 3.0 * (-u) + W3 * u2 - lastTerm) * tauI - W1rho * 3 * force_.x;
    temp[SW] = W2rho * (1.0 + 3.0 * (-u - v) + W3 * (-u - v) * (-u - v) - lastTerm) * tauI + W2rho * 3 * (-force_.x - force_.y);
    temp[S] = W1rho * (1.0 + 3.0 * (-v) + W3 * v2 - lastTerm) * tauI - W1rho * 3 * force_.y;
    temp[SE] = W2rho * (1.0 + 3.0 * (u - v) + W3 * (u - v) * (u - v) - lastTerm) * tauI + W2rho * 3 * (force_.x - force_.y);

    for (auto d : dirIter_) {
        const double tempD = distributions_[d];
        // compute non equilibirum and make relaxation
        distributions_[d] = tempD - tempD * tauI + temp[d];
    }

    // recalculate the velocity
    const double un = (distributions_[E] + distributions_[NE]
                       + distributions_[SE]
                       - (distributions_[NW] + distributions_[W]
                          + distributions_[SW])) * rhoI;
    const double vn = (distributions_[NE] + distributions_[N]
                       + distributions_[NW]
                       - (distributions_[SW] + distributions_[S]
                          + distributions_[SE])) * rhoI;
    assert(std::isfinite(un));
    assert(std::isfinite(vn));
    velocity_.x = un;
    velocity_.y = vn;

    // reseting the rho to add to zero as it is shifted to the distributions
    rhoToAdd_ = 0.0;
    // preparation for advect step
    localSwap();
}

void FluidSolver::advect() {
    std::swap(distributions_[E], physicalNode_.getPhysicalNeighbour(
                    W)->getFluidSolver().accessDistribution(W));
    std::swap(distributions_[NE], physicalNode_.getPhysicalNeighbour(
                    SW)->getFluidSolver().accessDistribution(SW));
    std::swap(distributions_[N], physicalNode_.getPhysicalNeighbour(
                    S)->getFluidSolver().accessDistribution(S));
    std::swap(distributions_[NW], physicalNode_.getPhysicalNeighbour(
                    SE)->getFluidSolver().accessDistribution(SE));
}

void FluidSolver::addForce(Field<double> f) {
    force_ += f;
}

void FluidSolver::writeSolver(std::ostream* const stream) {
    (*stream) << physicalNode_.getXPos() << '\t' << physicalNode_.getYPos();
    for (auto d : distributions_) {
        (*stream) << '\t' << d;
    }
    (*stream) << '\n';
}

void FluidSolver::loadSolver(std::stringstream* const stream) {
    int x, y;
    // reset of the stringstream
    stream->seekg(0);
    (*stream) >> x >> y;
    assert(physicalNode_.getXPos() == x && "The position does not match");
    assert(physicalNode_.getYPos() == y && "The position does not match");
    for (auto d : dirIter_) {
        (*stream) >> distributions_[d];
    }
}

void FluidSolver::resetForce() {
    force_.x = 0.0;
    force_.y = 0.0;
}

double& FluidSolver::accessDistribution(const Direction& dir) {
    return distributions_[dir];
}

void FluidSolver::rescaleDistributions(const double factor) {
    for (auto &it: this->distributions_) {
        it *= factor;
    }
}

void FluidSolver::setVelocity(Field<double> velocity) {
    velocity_ = velocity;
    assert(std::isfinite(this->velocity_.x));
    assert(std::isfinite(this->velocity_.y));
    velocityInit_ = true;
}

const Field<double>& FluidSolver::getVelocity() const {
    assert(std::isfinite(this->velocity_.x));
    assert(std::isfinite(this->velocity_.y));
    return velocity_;
}

double FluidSolver::getRho() const {
    return std::accumulate(distributions_.begin(),
            distributions_.end(), 0.0);
}

void FluidSolver::addMass(double mass) {
    this->rhoToAdd_ = 0.0;
    this->rhoToAdd_ += mass;
    LOG(UtilLib::logDEBUG4) << "mass added to the fluid. " << mass;
}

void FluidSolver::localSwap() {
    std::swap(distributions_[E], distributions_[W]);
    std::swap(distributions_[NE], distributions_[SW]);
    std::swap(distributions_[N], distributions_[S]);
    std::swap(distributions_[NW], distributions_[SE]);
}


FluidSolver::FluidSolver(const nodes::PhysicalNode& physicalNode)
    : AbstractSolver(),
      physicalNode_(physicalNode),
      distributions_(std::array<double,
                      9> {{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }
              }),
      velocityInit_(false),
      rhoToAdd_(0.0)
{}

DirectionIterator const FluidSolver::dirIter_ = DirectionIterator();
}
}  // end namespace
