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
#include <LbmLib/include/solver/BoundarySolverFactory.hpp>
#include <LbmLib/include/solver/BoundaryAbstractSolver.hpp>
#include <iomanip>
#include <map>
#include <string>
#include <vector>
#include <cassert>
#include <sstream>

namespace LbmLib {
namespace nodes {
BoundaryNode::BoundaryNode(
        double x,
        double y,
        const std::map<std::string, std::vector<std::string> >& connectionType,
        unsigned int domainId) : LagrangianPoint(x, y),
                                 domainId_(domainId),
                                 physicalNeighbour_(nullptr) {
    // make the connection between the boundary solvers and the cde solvers
    for (auto solver : connectionType) {
        this->addBoundarySolver(solver.first);
        this->getBoundarySolver(solver.first).connectToCDESolvers(solver.second);
    }
}

unsigned int BoundaryNode::getDomainIdentifier() const {
    return this->domainId_;
}

BoundaryNode::~BoundaryNode() {
    for (const auto& i : boundarySolvers_) {
        delete i.second;
    }
}

void BoundaryNode::setPhysicalNeighbours(
        PhysicalNode* const physicalNode,
        const Direction& dir) {
    assert(physicalNode != nullptr);
    physicalNeighbour_ = physicalNode;
    directionToPhysicalNeighbour_ = dir;
    physicalNode->setBoundaryNeighbour(this, getInverseDirection(dir));
    assert(physicalNeighbour_->getBoundaryNeighbour(getInverseDirection(
                            dir)) == this);
    assert(physicalNeighbour_ == physicalNode);
}

Direction BoundaryNode::getDirectionToNeighbour() const {
    return directionToPhysicalNeighbour_;
}

PhysicalNode* BoundaryNode::getPhysicalNeighbour() const {
    return physicalNeighbour_;
}

std::string BoundaryNode::getType() const {
    return std::string("BoundaryNode");
}

solver::BoundaryAbstractSolver& BoundaryNode::getBoundarySolver(
        const std::string& name) {
    auto retVal = boundarySolvers_.find(name);
    if (retVal == boundarySolvers_.end()) {
        std::stringstream error;
        error << "The CDE Solver with the name " << name <<
        " does not exists. You need to add them first\n";
        throw UtilLib::Exception(error.str());
    }
    return *retVal->second;
}

std::map<std::string,
        solver::BoundaryAbstractSolver*>& BoundaryNode::getBoundarySolvers() {
    return boundarySolvers_;
}

void BoundaryNode::addBoundarySolver(const std::string& name) {
    if (boundarySolvers_.find(name) != boundarySolvers_.end()) {
        std::stringstream error;
        error << "The Boundary Solver with the name " << name <<
        " already exists. You cannot add the same twice\n";
        throw UtilLib::Exception(error.str());
    }
    boundarySolvers_[name] =
        solver::BoundarySolverFactory::instance().createObject(name);
    boundarySolvers_[name]->connectBoundaryNode(this);
}

void BoundaryNode::dumpNode(std::ostream* oStream) const {
    double localId = 10000000.0 * getXPos() + 10000.0 * static_cast<double>(getYPos());
    (*oStream) << std::setprecision(12) << localId << "[pos=\"" << getXPos() <<
    "," << getYPos() << std::setprecision(5) <<
    "!\",shape=triangle, label=\"" << getXPos() << ";" << getYPos() <<
    "\"];" << std::endl;

    double nodeId1 = 10000000.0 * physicalNeighbour_->getXPos() + 10000.0 *
        static_cast<double>(physicalNeighbour_->getYPos());
    (*oStream) << std::setprecision(12) << nodeId1 << std::setprecision(5) <<
    "[pos=\"" << physicalNeighbour_->getXPos() << "," <<
    physicalNeighbour_->getYPos() << "!\",shape=circle, label=\"" <<
    physicalNeighbour_->getXPos() << ";" <<
    physicalNeighbour_->getYPos() << "\"];" << std::endl;

    (*oStream) << std::setprecision(12) << localId << "->" << nodeId1 <<
    std::endl;
}
}   // end namespace
}  // end namespace
