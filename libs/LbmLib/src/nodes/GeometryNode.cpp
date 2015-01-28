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
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <UtilLib/include/geometry/Rectangle.hpp>
#include <LbmLib/include/Constants.hpp>
#include <UtilLib/include/Log.hpp>
#include <LbmLib/include/Field.hpp>
#include <string>
#include <random>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <cassert>
#include <omp.h>

namespace LbmLib {
namespace geometry {
class Connection;
}
}

namespace LbmLib {
namespace nodes {
namespace {

constexpr double halfPi = M_PI / 2.0; ///< pi/2.0
constexpr double doublePi = 2.0 * M_PI; ///< 2.0*pi

/**
 * @brief fastCos Fast cos() implementation according to http://lab.polygonal.de/?p=205
 * @param x
 * @return Cos(x)
 */
inline double fastCos(const double x) {
    //compute cosine: sin(x + PI/2) = cos(x)
    double cos;
    double tempx = x + halfPi;
    if (tempx >  M_PI) {
        tempx -= doublePi;
    }
    if (tempx < .00) {
        cos = 1.27323954 * tempx + 0.405284735 * tempx * tempx;
        if (cos < 0.0) {
            cos = 0.225 * (cos *-cos - cos) + cos;
        }
        else {
            cos = 0.225 * (cos * cos - cos) + cos;
        }
    }
    else {
        cos = 1.27323954 * tempx - 0.405284735 * tempx * tempx;
        if (cos < 0.0) {
            cos = 0.225 * (cos *-cos - cos) + cos;
        }
        else {
            cos = 0.225 * (cos * cos - cos) + cos;
        }
    }
return cos;
}

/**
 * @brief calculateDiscreteDeltaDirac
 * @param xpos The y position.
 * @param ypos The x position.
 * @return The discrete delta dirac value.
 */
inline double calculateDiscreteDeltaDirac(const double xpos,const double ypos) {
    return (
                (1.0 + fastCos(halfPi * xpos)) *
                (1.0 + fastCos(halfPi * ypos))
                ) / 16.0;
}
}

// hack to make sure that the connections works. difference between oldPos to new must be greater than 1
GeometryNode::GeometryNode(
        double x,
        double y,
        unsigned int id) : LagrangianPoint(x, y),
                           oldPosition_(x - 5.0, y - 5.0),
                           id_(id),
                           neighbourPhysicalNodes_(std::array<PhysicalNode*,
                                           16> {{nullptr, nullptr, nullptr,
                                        nullptr, nullptr, nullptr,
                                        nullptr, nullptr, nullptr,
                                        nullptr, nullptr, nullptr,
                                        nullptr, nullptr, nullptr,
                                        nullptr}}),
                           connections_(std::array<std::shared_ptr<LbmLib::geometry::Connection>,
                                        2> {{nullptr,
                                        nullptr}})

{
    perturbatePosition();
}

GeometryNode::~GeometryNode()
{
    std::stringstream message;
    message << "GeometryNode with nodeid=" << this->getId() << " finally destroyed.";
    LOG(UtilLib::logDEBUG1) << message.str().c_str();
}

void GeometryNode::writeNode(std::ostream* stream) {
    (*stream) << id_ << '\t' << getXPos() << '\t' << getYPos() << '\n';
}

void GeometryNode::addForce(Field<double> f) {
    force_ += f;
}

void GeometryNode::addPhysicalNode(
        PhysicalNode* const physicalNode,
        unsigned int pos) {
    assert(pos < 16);
    assert(physicalNode != nullptr);
    neighbourPhysicalNodes_[pos] = physicalNode;
    assert(physicalNode == neighbourPhysicalNodes_[pos]);
}

void GeometryNode::setPos(
        double x,
        double y) {
#pragma omp critical
    {
    LagrangianPoint::setPos(x, y);
    perturbatePosition();
    }
}

void GeometryNode::perturbatePosition() {
    const int seed = 1; // for debugging
    static std::random_device rd;
    //static std::mt19937 gen(rd());
    static std::mt19937 gen(seed);
    static std::uniform_real_distribution<> dis(-PERTURBATION, PERTURBATION);
    double temprand;
    if (std::abs(static_cast<int>(getXPos()) - getXPos()) <= EPSILON) {
        std::stringstream message;
        message << std::setprecision(12)
                << "Perturbation of GeometryNode (ID="<<this->getId()<<") "
                << "(" << this->getXPos() << "," << this->getYPos() << ")"
                << "->";
        temprand = 0.0;
        while(std::abs(temprand) < 5.0*EPSILON) {
            temprand = dis(gen);
        }
        LagrangianPoint::setPos(getXPos() + temprand, getYPos());
        message << "(" << this->getXPos() << "," << this->getYPos() << ")";
        LOG(UtilLib::logINFO) << message.str().c_str();
    }
    if (std::abs(static_cast<int>(getYPos()) - getYPos()) <= EPSILON) {
        std::stringstream message;
        message << std::setprecision(12)
                << "Perturbation of GeometryNode (ID="<<this->getId()<<") "
                << "(" << this->getXPos() << "," << this->getYPos() << ")"
                << "->";
        temprand = 0.0;
        while(std::abs(temprand) < 5.0*EPSILON) {
            temprand = dis(gen);
        }
        LagrangianPoint::setPos(getXPos(), getYPos() +  temprand);
        message << "(" << this->getXPos() << "," << this->getYPos() << ")";
        LOG(UtilLib::logINFO) << message.str().c_str();
    }

    if (std::abs(getYPos() - std::floor(getYPos()) -
                (getXPos() - std::floor(getXPos()))) <= EPSILON) {
        std::stringstream message;
        message << std::setprecision(12);
        message << "Perturbation of GeometryNode (ID="<<this->getId()<<") ";
        message << "(" << this->getXPos() << "," << this->getYPos() << ")";
        message << "->";
        LagrangianPoint::setPos(getXPos() +  dis(gen),
                getYPos() +  dis(gen));
        message << "(" << this->getXPos() << "," << this->getYPos() << ")";
        LOG(UtilLib::logINFO) << message.str().c_str();
    }

    if (std::abs(std::ceil(getYPos()) - getYPos() -
                (getXPos() - std::floor(getXPos()))) <= EPSILON) {
        std::stringstream message;
        message << std::setprecision(12);
        message << "Perturbation of GeometryNode (ID="<<this->getId()<<") ";
        message << "(" << this->getXPos() << "," << this->getYPos() << ")";
        message << "->";
        LagrangianPoint::setPos(getXPos() +  dis(gen),
                getYPos() +  dis(gen));
        message << "(" << this->getXPos() << "," << this->getYPos() << ")";
        LOG(UtilLib::logINFO) << message.str().c_str();
    }
}

void GeometryNode::collectVelocity() {
    Field<double> velocity;
    for (const auto& node : neighbourPhysicalNodes_) {
        Field<double> dist = getDistanceField(*this, *node);
        velocity += calculateDiscreteDeltaDirac(dist.x,dist.y) *
            node->getFluidSolver().getVelocity();
    }
    assert(std::isfinite(velocity.x));
    assert(std::isfinite(velocity.y));
    this->setVelocity(velocity);
}

void GeometryNode::distributeForce() {
    Field<double> dist;
#pragma omp parallel for schedule(dynamic) private(dist)
    for (size_t it=0;it<this->neighbourPhysicalNodes_.size();++it) {
        dist = nodes::getDistanceField(*this, *this->neighbourPhysicalNodes_[it]);
        this->neighbourPhysicalNodes_[it]->getFluidSolver().addForce(calculateDiscreteDeltaDirac(dist.x,dist.y) * force_);
    }
}

void GeometryNode::setForce(Field<double> f) {
    force_ = f;
}

std::string GeometryNode::getType() const {
    return std::string("GeometryNode");
}

void GeometryNode::move() {
    this->oldPosition_.x = this->getXPos();
    this->oldPosition_.y = this->getYPos();
    // assume delta_time=1
    assert(std::isfinite(this->getXPos()));
    assert(std::isfinite(this->getYPos()));
    assert(std::isfinite(this->getXVelocity()));
    assert(std::isfinite(this->getYVelocity()));
    setPos(getXPos() + getXVelocity(), getYPos() + getYVelocity());
}

bool GeometryNode::isUpdateNeeded() const {
    UtilLib::geometry::Rectangle rec = UtilLib::geometry::Rectangle(std::ceil(
                        this->oldPosition_.y), std::floor(this->oldPosition_.y),
                std::floor(this->oldPosition_.x), std::ceil(this->oldPosition_.x));
    return !rec.pointWithinBounds(getXPos(), getYPos());
}

unsigned int GeometryNode::getId() const {
    return id_;
}

unsigned int GeometryNode::getDomainIdOfAdjacentConnections() const
{
    unsigned int temp;
    assert(this->getConnection<0>() != nullptr);
    assert(this->getConnection<1>() != nullptr);

    temp = this->getConnection<0>()->getDomainIdentifier();
    if (this->getConnection<1>()->getDomainIdentifier() != temp) {
        std::stringstream message;
        message << std::setprecision(12) <<
                   "GeometryNode ID="<<this->getId()<<") " <<
                   "(" << this->getXPos() << "," << this->getYPos() << ")" <<
                   " has adjacent connections with non-identical domainIdentifiers";
        LOG(UtilLib::logINFO) << message.str().c_str();
    }
    return temp;
}

}
}  // end namespace
