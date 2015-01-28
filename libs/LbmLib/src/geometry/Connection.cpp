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
#include <LbmLib/include/geometry/Connection.hpp>
#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>

#include <LbmLib/include/Constants.hpp>

#include <UtilLib/include/Log.hpp>
#include <algorithm>
#include <random>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <string>
#include <utility>
#include <vector>
#include <omp.h>

namespace LbmLib {
namespace geometry {
class GeometryNode;

/**
 * @todo: self-registration of connection
 */
Connection::Connection(
        std::shared_ptr<nodes::GeometryNode> const p1,
        std::shared_ptr<nodes::GeometryNode> const p2,
        const std::map<std::string, std::vector<std::string> > connectionType,
        unsigned int domainId) : points_(std::make_pair(p1, p2)),
                                 connectionType_(connectionType),
                                 domainIdentifier_(domainId)
{
    //p1->setConnection<0>(std::make_shared<Connection>(this));
    //p2->setConnection<1>(std::shared_ptr<Connection>(this));
}

Connection::~Connection()
{
    /**
      * @todo remove these lines. i don't know when the destructor is called -> random behaviour.
      */
}

namespace {
/**
 * @brief helper function to get y
 * @param m the slope
 * @param x the x value
 * @param n the axis intercept
 * @return the y
 */
double getYPos(
        double m,
        double x,
        double n) {
    return m * x + n;
}

/**
 * @brief helper function to get x
 * @param m the slope
 * @param y the y value
 * @param n the axis intercept
 * @return the x
 */
double getXPos(
        double m,
        double y,
        double n) {
    return (y - n) / m;
}

/**
 * @brief helper to get gradient
 * @param x1 the x1
 * @param x2 the x2
 * @param y1 the y2
 * @param y2 the y2
 * @return the slope
 */
double getGradient(
        double x1,
        double x2,
        double y1,
        double y2) {
    return (y2 - y1) / (x2 - x1);
}

/**
 * @brief helper to get axis intercept
 * @param x1 the x1
 * @param x2 the x2
 * @param y1 the y2
 * @param y2 the y2
 * @return the axis intercept
 */
double getAxisIntercept(
        double x1,
        double x2,
        double y1,
        double y2) {
    return y1 - (y2 - y1) / (x2 - x1) * x1;
}

/**
 * @brief helper to get axis intercept
 * @param x the x
 * @param y the y
 * @param m the slope
 * @return the axis intercept
 */
double getAxisIntercept(
        double x,
        double y,
        double m) {
    return y - m * x;
}
}

Field<double> Connection::calculateVelocity(const nodes::BoundaryNode& bNode) {
    const double l = getDistance(*points_.first,bNode);
    const double L = getDistance(*points_.second, *points_.first);
    const double vx1 = this->points_.first->getXVelocity();
    const double vy1 = this->points_.first->getYVelocity();
    const double vx2 = this->points_.second->getXVelocity();
    const double vy2 = this->points_.second->getYVelocity();
    const double vx = (vx2-vx1) * l/L + vx1;
    const double vy = (vy2-vy1) * l/L + vy1;
    assert(std::isfinite(l));
    assert(std::isfinite(L));
    assert(std::isfinite(vx1));
    assert(std::isfinite(vy1));
    assert(std::isfinite(vx2));
    assert(std::isfinite(vy2));
    assert(std::isfinite(vx));
    assert(std::isfinite(vy));
    Field<double> f;
    f.x = vx;
    f.y = vy;
    return f;
    //return ((points_.second->getVelocity() - points_.first->getVelocity()) / getDistance(*points_.second, *points_.first)) * getDistance(*points_.first,bNode) + points_.first->getVelocity();
}

void Connection::writeConnection(std::ofstream* const stream) {
    (*stream) << points_.first->getId() << '\t' << points_.second->getId() <<
        '\t' << domainIdentifier_;
    for (const auto& bSolver : connectionType_) {
        for (const auto& cdeSolver : bSolver.second) {
            (*stream) << '\t' << bSolver.first << '\t' << cdeSolver;
        }
    }
    (*stream) << '\n';
}

void Connection::insertBoundaryNode(nodes::BoundaryNode* const newBoundaryNode,
        nodes::PhysicalNode* const physicalNode,
        const Direction& d,
        std::unordered_set<nodes::BoundaryNode *> &boundaryNodes) {
    assert(newBoundaryNode != nullptr);
    assert(physicalNode != nullptr);
    nodes::BoundaryNode* old = physicalNode->getBoundaryNeighbour(
                getInverseDirection(d));

    // not a boundary node yet
    if (old == nullptr) {
        Field<double> ftemp = calculateVelocity(*newBoundaryNode);
        assert(std::isfinite(ftemp.x));
        assert(std::isfinite(ftemp.y));
        newBoundaryNode->setVelocity(ftemp);
#pragma omp critical // boundaryNodes is not thread safe
        {
        boundaryNodes.insert(newBoundaryNode);
        }
        newBoundaryNode->setPhysicalNeighbours(physicalNode, d);
    } else if (getSquaredDistance(*physicalNode,
                                  *newBoundaryNode) <
               getSquaredDistance(*physicalNode, *old)) {
        // it has a boundary node but the new is closer
        Field<double> ftemp = calculateVelocity(*newBoundaryNode);
        assert(std::isfinite(ftemp.x));
        assert(std::isfinite(ftemp.y));
        newBoundaryNode->setVelocity(ftemp);

#pragma omp critical // boundaryNodes is not thread safe
        {
        boundaryNodes.insert(newBoundaryNode);
        }
#pragma omp critical // boundaryNodes is not thread safe
        {
        boundaryNodes.erase(old);
        }
        newBoundaryNode->setPhysicalNeighbours(physicalNode, d);
    } else {
        delete newBoundaryNode; // the old boundary node is closer
    }
}

void Connection::generateBoundaryNodes(const std::vector<std::vector<nodes::PhysicalNode*> >& physicalNodes,
        std::unordered_set<nodes::BoundaryNode *> &boundaryNodes) {
    assert(std::isfinite(this->points_.first->getXPos()));
    assert(std::isfinite(this->points_.first->getYPos()));
    assert(std::isfinite(this->points_.second->getXPos()));
    assert(std::isfinite(this->points_.second->getYPos()));
    double maxX = std::max(this->points_.first->getXPos(), this->points_.second->getXPos());
    double maxY = std::max(this->points_.first->getYPos(), this->points_.second->getYPos());
    double minX = std::min(this->points_.first->getXPos(), this->points_.second->getXPos());
    double minY = std::min(this->points_.first->getYPos(), this->points_.second->getYPos());

    // compute slope:
    double m = getGradient(points_.first->getXPos(),
                points_.second->getXPos(),
                points_.first->getYPos(), points_.second->getYPos());

    // compute intercept:
    double n = getAxisIntercept(
                points_.first->getXPos(),
                points_.second->getXPos(),
                points_.first->getYPos(), points_.second->getYPos());

    if (std::isfinite(m)) {
        // with vertical lattice lines:
        for (int i = std::ceil(minX); i <= std::floor(maxX); i++) {
            nodes::BoundaryNode* bpS; // south
            nodes::BoundaryNode* bpN; // north
            if (points_.first->getXPos() < points_.second->getXPos()) {
                // the north is inside
                bpN = new nodes::BoundaryNode(i, getYPos(m,
                                    i,
                                    n), connectionType_,
                                              this->domainIdentifier_);
                bpS = new nodes::BoundaryNode(i, getYPos(m,
                                    i,
                                    n), connectionType_,
                                              0);
            } else {
                // the south is inside
                bpN = new nodes::BoundaryNode(i, getYPos(m,
                                    i,
                                    n), connectionType_,
                                              0);
                bpS = new nodes::BoundaryNode(i, getYPos(m,
                                    i,
                                    n), connectionType_,
                                              this->domainIdentifier_);
            }

            nodes::PhysicalNode* fluidS =
                physicalNodes[std::floor(getYPos(m, i, n))][i];
            nodes::PhysicalNode* fluidN =
                physicalNodes[std::ceil(getYPos(m, i, n))][i];

            this->insertBoundaryNode(bpS, fluidS, S, boundaryNodes);
            this->insertBoundaryNode(bpN, fluidN, N, boundaryNodes);
        }

        // with horizontal lattice lines:
        for (int i = std::ceil(minY); i <= std::floor(maxY); i++) {
            nodes::BoundaryNode* bpE;
            nodes::BoundaryNode* bpW;
            if (points_.first->getYPos() > points_.second->getYPos()) {
                // east is inside
                bpW = new nodes::BoundaryNode(getXPos(m,
                                    i,
                                    n), i, connectionType_,
                                              0);
                bpE = new nodes::BoundaryNode(getXPos(m,
                                    i,
                                    n), i, connectionType_,
                                              this->domainIdentifier_);
            } else {
                // west is inside
                bpW = new nodes::BoundaryNode(getXPos(m,
                                    i,
                                    n), i, connectionType_,
                                              this->domainIdentifier_);
                bpE = new nodes::BoundaryNode(getXPos(m,
                                    i,
                                    n), i, connectionType_,
                                              0);
            }
            nodes::PhysicalNode* fluidW =
                physicalNodes[i][std::floor(getXPos(m, i, n))];
            nodes::PhysicalNode* fluidE =
                physicalNodes[i][std::ceil(getXPos(m, i, n))];

            this->insertBoundaryNode(bpW, fluidW, W, boundaryNodes);
            this->insertBoundaryNode(bpE, fluidE, E, boundaryNodes);
        }
    } else {
        std::stringstream message;
        message << std::setprecision(12);
        message << "This can only happen if something went wrong with the perturbation of the connection. ";
        message << "p1.x="<<points_.first->getXPos()<<", p1.y="<<points_.first->getYPos();
        message << ", p2.x="<<points_.second->getXPos()<<", p2.y="<<points_.second->getYPos();
#pragma omp critical
        lbm_fail(message.str().c_str());
    }
}

std::pair<const std::shared_ptr<nodes::GeometryNode>,
        const std::shared_ptr<nodes::GeometryNode> > Connection::
  getGeometryNodes() const {
    return points_;
}

void Connection::perturbConnection() {
    const int seed = 1; // for debugging
    static std::random_device rd;
    //static std::mt19937 gen(rd());
    static std::mt19937 gen(seed);
    static std::uniform_real_distribution<> dis(-PERTURBATION, PERTURBATION);
    double temprand;
    if (std::abs(points_.first->getXPos() - points_.second->getXPos()) <
        EPSILON) {
        std::stringstream message;
        message << std::setprecision(12)
                << "Perturbation of Connection ("
                << this->points_.first->getId() << "->" << this->points_.second->getId()<<") " // ID
                << "(" << this->points_.first->getXPos() << "," << this->points_.first->getYPos() << ")" // first GeometryNode
                << "->"
                << "(" << this->points_.second->getXPos() << "," << this->points_.second->getYPos() << ") " // second GeometryNode
                << " to ";
        temprand = 0.0;
        while(std::abs(temprand) < 5.0*EPSILON) {
            temprand = dis(gen);
        }
        points_.first->setPos(
                    points_.first->getXPos() + temprand,
                    points_.first->getYPos());
        message << "(" << this->points_.first->getXPos() << "," << this->points_.first->getYPos() << ")" // first GeometryNode
                << "->"
                << "(" << this->points_.second->getXPos() << "," << this->points_.second->getYPos() << ") "; // second GeometryNode
#pragma omp critical
        LOG(UtilLib::logINFO) << message.str().c_str();
    }

    if (std::abs(points_.first->getYPos() - points_.second->getYPos()) <
        EPSILON) {
        std::stringstream message;
        message << std::setprecision(12)
                << "Perturbation of Connection ("
                << this->points_.first->getId() << "->" << this->points_.second->getId()<<") " // ID
                << "(" << this->points_.first->getXPos() << "," << this->points_.first->getYPos() << ")" // first GeometryNode
                << "->"
                << "(" << this->points_.second->getXPos() << "," << this->points_.second->getYPos() << ") " // second GeometryNode
                << " to ";
        temprand = 0.0;
        while(std::abs(temprand) < 5.0*EPSILON) {
            temprand = dis(gen);
        }
        points_.first->setPos(
                points_.first->getXPos(),
                points_.first->getYPos() + temprand);
        message << "(" << this->points_.first->getXPos() << "," << this->points_.first->getYPos() << ")" // first GeometryNode
                << "->"
                << "(" << this->points_.second->getXPos() << "," << this->points_.second->getYPos() << ") "; // second GeometryNode

#pragma omp critical
        LOG(UtilLib::logINFO) << message.str().c_str();
    }
}

double Connection::getLength() const {
    return std::sqrt((this->points_.first->getXPos() - this->points_.second->getXPos()) *
                     (this->points_.first->getXPos() - this->points_.second->getXPos()) +
                     (this->points_.first->getYPos() - this->points_.second->getYPos()) *
                     (this->points_.first->getYPos() - this->points_.second->getYPos()));
}

unsigned int Connection::getDomainIdentifier() const {
    return this->domainIdentifier_;
}

const std::map<std::string, std::vector<std::string> > Connection::getBoundaryConditionDescriptor() const {
    return this->connectionType_;
}

void Connection::setDomainIdentifier(const unsigned int domainIdentifier)
{
    this->domainIdentifier_ = domainIdentifier;
}


}   // end namespace
}  // end namespace
