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
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>
#include <LbmLib/include/solver/CDESolver/CDESolverFactory.hpp>
#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Log.hpp>
#include <iomanip>
#include <cassert>
#include <sstream>
#include <string>
#include <map>
#include <array>
#include <vector>
#include <algorithm>

namespace LbmLib {
namespace nodes {
PhysicalNode::PhysicalNode(
        int x,
        int y) : EulerianPoint(x, y),
                 fluidSolver_(solver::FluidSolver(*this)),
                 neighbourNodes_(std::array<PhysicalNode*,
                                 9> {{nullptr, nullptr, nullptr, nullptr,
                                      nullptr, nullptr, nullptr,
                                      nullptr, nullptr}
                         }),
                 boundaryNodes_(std::array<BoundaryNode*,
                                 5> {{nullptr, nullptr, nullptr, nullptr,
                                      nullptr }
                         }),
                 domainIdentifier_(0),
                 cellType_(0)
{}

PhysicalNode::~PhysicalNode() {
    for (const auto& i : cdeSolvers_) {
        delete i;
    }
}

void PhysicalNode::addCDESolver(const std::string& cdeSolverName) {
    struct Contains {
        const std::string d;
        explicit Contains(std::string s) : d(s) {}

        bool operator()(solver::CDEAbstractSolver* n) const { return n->getName()
                                                                     == d; }
    };


    if (std::any_of(this->cdeSolvers_.begin(), this->cdeSolvers_.end(),
                Contains(cdeSolverName))) {
        std::stringstream error;
        error << "The CDE Solver with the name " << cdeSolverName <<
        " already exists. You cannot add the same twice\n";
        lbm_fail(error.str());
    }
    this->cdeSolvers_.push_back(solver::CDESolverFactory::instance().createObject(
                    cdeSolverName));

    this->cdeSolvers_.back()->initCDESolver(this, this->cdeSolvers_.size() - 1);
}

void PhysicalNode::setPhysicalNeighbour(
        PhysicalNode* const node,
        const Direction& d) {
    assert(node != nullptr);
    neighbourNodes_[d] = node;
    assert(node == neighbourNodes_[d]);
}

void PhysicalNode::setBoundaryNeighbour(
        BoundaryNode* const boundaryNode,
        const Direction& d) {
    assert(boundaryNode != nullptr);
    boundaryNodes_[d] = boundaryNode;
    assert(boundaryNode == boundaryNodes_[d]);
}

BoundaryNode* PhysicalNode::getBoundaryNeighbour(const Direction& d) const {
    return boundaryNodes_[d];
}

PhysicalNode* PhysicalNode::getPhysicalNeighbour(const Direction& d) const {
    return neighbourNodes_[d];
}

void PhysicalNode::updateDomainIdentifier() {
    unsigned int domainTemp = this->domainIdentifier_;
    for (auto bd : this->boundaryNodes_) {
        if (bd != nullptr) {
            domainTemp = bd->getDomainIdentifier();
        }
    }

    if (this->domainIdentifier_ != domainTemp) {
        LOG(UtilLib::logINFO) << "PhysicalNode at (" <<this->getXPos() << ","
                              << this->getYPos() << ") " << "changed domainID from "
                              << this->domainIdentifier_ << " to " << domainTemp;
        this->domainIdentifier_ = domainTemp;
        this->reinitialiseCDESolvers();
    }
}

unsigned int PhysicalNode::getDomainIdentifier() const {
    return domainIdentifier_;
}

void PhysicalNode::setDomainIdentifier(unsigned int domainIdentifier) {
    domainIdentifier_ = domainIdentifier;
}

unsigned int PhysicalNode::getCellType () const {
    return this->cellType_;
}

void PhysicalNode::setCellType(unsigned int celltype) {
    this->cellType_ = celltype;
}

void PhysicalNode::reinitialiseCDESolvers() {
    for (auto cdeSolver : cdeSolvers_) {
        cdeSolver->reinitialise();
    }
}

void PhysicalNode::resetBoundaryNodes() {
    boundaryNodes_ =
        std::array<BoundaryNode*, 5> {{nullptr, nullptr, nullptr, nullptr,
                                       nullptr}};
}

std::string PhysicalNode::getType() const {
    return std::string("PhysicalNode");
}

const solver::FluidSolver& PhysicalNode::getFluidSolver() const {
    return fluidSolver_;
}

solver::FluidSolver& PhysicalNode::getFluidSolver() {
    return fluidSolver_;
}

solver::CDEAbstractSolver& PhysicalNode::getCDESolver(size_t id) const {
    return *cdeSolvers_[id];
}

solver::CDEAbstractSolver& PhysicalNode::getCDESolverSlow(
        const std::string& name) const {
    for (const auto& cdeSolver : cdeSolvers_) {
        if (cdeSolver->getName() == name) {
            return *cdeSolver;
        }
    }
    std::stringstream error;
    error << "The CDE Solver with the name " << name <<
    " does not exists. You need to add them first\n";
    throw UtilLib::Exception(error.str());
}

std::vector<solver::CDEAbstractSolver*>& PhysicalNode::getCDESolvers() {
    return this->cdeSolvers_;
}

void PhysicalNode::dumpNode(std::ostream* oStream) const {
    double localId = 10000000.0 * getXPos() + 10000.0 *
        static_cast<double>(getYPos());
    (*oStream) << std::setprecision(12) << localId << "[pos=\"" << getXPos() <<
    "," << getYPos() << std::setprecision(5) <<
    "!\",shape=circle, label=\"" << domainIdentifier_ << "\"];" <<
    std::endl;
    int i = 0;
    for (auto bt : boundaryNodes_) {
        if (bt != nullptr) {
            double nodeId1 = 10000000.0 * bt->getXPos() + 10000.0 *
                static_cast<double>(bt->getYPos());
            (*oStream) << std::setprecision(12) << nodeId1 << std::setprecision(
                    5) << "[pos=\"" << bt->getXPos() << "," << bt->getYPos() <<
            "!\",shape=triangle, label=\"" << bt->getXPos() << ";" <<
            bt->getYPos() << "\"];" << std::endl;

            (*oStream) << std::setprecision(12) << localId << "->" <<
            nodeId1 << std::setprecision(5) << " [ label=\"" << i <<
            "\" ];" << std::endl;
        }
        i++;
    }
}
}
}  // end namespace
