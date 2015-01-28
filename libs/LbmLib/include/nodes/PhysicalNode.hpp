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
#ifndef FLUIDPOINT_HPP
#define FLUIDPOINT_HPP


#include <LbmLib/include/nodes/EulerianPoint.hpp>
#include <LbmLib/include/Direction.hpp>
#include <LbmLib/include/solver/FluidSolver/FluidSolver.hpp>
#include <array>
#include <vector>
#include <string>
#include <iostream>
#include <memory>

namespace LbmLib {
namespace solver {
class CDEAbstractSolver;
}
namespace nodes {
class BoundaryNode;

/**
 * @brief class representing a physical node
 */
class PhysicalNode : public EulerianPoint {
 public:
    /**
     * @brief PhysicalNode constructs a physical node
     * @param x the x pos
     * @param y the y pos
     */
    explicit PhysicalNode(
            int x,
            int y);
    /**
     * @brief ~PhysicalNode Destructor
     */
    ~PhysicalNode();
    /**
     * @brief setPhysicalNeighbour Setter for the neighbour fluid point
     * @param node A pointer to the neighbour fluid point
     * @param d The direction
     */
    void setPhysicalNeighbour(
            PhysicalNode* const node,
            const Direction& d);

    /**
     * @brief setBoundaryNeighbour Setter for the neighbour boundary point
     * @param boundaryNode A pointer to the neighbour boundary point
     * @param d The direction
     */
    void setBoundaryNeighbour(
            BoundaryNode* const boundaryNode,
            const Direction& d);

    /**
     * @brief getBoundaryNeighbour Getter method to access the Boundary Neighbour
     * @param d The direction of the neighbour
     * @return A Pointer to the boundary neighbour
     */
    BoundaryNode* getBoundaryNeighbour(const Direction& d) const;

    /**
     * @brief getPhysicalNeighbour Getter method to access the Physical Neighbour
     * @param d The direction of the neighbour
     * @return A Pointer to the physical neighbour
     */
    PhysicalNode* getPhysicalNeighbour(const Direction& d) const;

    /**
     * @brief resetBoundaryNodes Resets the boundary nodes to nullptr
     */
    void resetBoundaryNodes();

    /**
     * @brief getType The type of a node class
     * @return Returns the class Name of the point
     */
    virtual std::string getType() const;

    /**
     * @brief dumpNode dumps the node for dot
     * @param oStream The stream this node is dumped to
     */
    void dumpNode(std::ostream* oStream) const;

    /**
     * @brief getFluidSolver Const getter method for the fluid Solver
     * @return  The Fluid Solver on this node.
     */
    const solver::FluidSolver& getFluidSolver() const;

    /**
     * @brief getFluidSolver Getter method for the fluid Solver
     * @return  The Fluid Solver on this node.
     */
    solver::FluidSolver& getFluidSolver();

    /**
     * @brief getCDESolver Getter method for the cde Solver
     * @param id The id of the CDE solver
     * @return The cde solver on this node
     */
    solver::CDEAbstractSolver& getCDESolver(size_t id) const;

    /**
     * @brief getCDESolverSlow Getter method for the cde Solver
     * @attention this method is slow prefere the getCDESolver method above
     * @param name The name of the CDE solver
     * @return The cde solver on this node
     */
    solver::CDEAbstractSolver& getCDESolverSlow(const std::string& name) const;

    /**
     * @brief getCDESolvers Getter method for all CDE Solvers
     * @return All CDE Solvers
     */
    std::vector<solver::CDEAbstractSolver*>& getCDESolvers();

    /**
     * @brief addCDESolver Adds a CDESolver to this node
     * @param cdeSolverName The name of the cde Solver added
     */
    void addCDESolver(const std::string& cdeSolverName);

    /**
     * @brief updateDomainIdentifier updates the domain Identifier of this node. If the domain changes it returns true otherwise false
     * if domain identifier has changed, it will be logged at debug 3 level
     */
    void updateDomainIdentifier();

    /**
     * @brief getter for the Domain Identifier of this node
     * @return the Domain Identifier of this node
     */
    unsigned int getDomainIdentifier() const;

    /**
     * @brief setDomainIdentifier setter for the domain identifier
     * @param domainIdentifier the new identifier
     */
    void setDomainIdentifier(unsigned int domainIdentifier);

    /**
     * @brief getter for the cell type of this node
     * @return the cell type of this node
     */
    unsigned int getCellType() const;

    /**
     * @brief setCellType setter for the celltype
     * @param celltype the new celltype
     */
    void setCellType(unsigned int celltype);

 private:
    /**
     * @brief reinitialiseCDESolvers reinitialises the CDE Solvers when the physical node has switched domain.
     */
    void reinitialiseCDESolvers();

    /**
     * @brief fluidSolver_ The fluid solver on this node
     */
    solver::FluidSolver fluidSolver_;
    /**
     * @brief cdeSolvers_ The list of the cde solvers
     */
    std::vector<solver::CDEAbstractSolver*> cdeSolvers_;

    /**
     * @brief neighbourNodes_ Pointer to the neighbour nodes, at the beginning these are initialised to nullptr
     */
    std::array<PhysicalNode*, 9> neighbourNodes_;

    /**
     * @brief boundaryNodes_ Pointer to possible boundary nodes (0...4).
     **/
    std::array<BoundaryNode*, 5> boundaryNodes_;

    /**
     * @brief domainIdentifier_ The identifier of the domain the node belongs to
     */
    unsigned int domainIdentifier_;

    /**
     * @brief cellType_ The cell type.
     */
    unsigned int cellType_;
};
}   // end namespace
}  // end namespace

#endif  // FLUIDPOINT_HPP
