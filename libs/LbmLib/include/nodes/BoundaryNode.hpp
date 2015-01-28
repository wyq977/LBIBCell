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
#ifndef INTERSECTIONPOINT_HPP
#define INTERSECTIONPOINT_HPP


#include <LbmLib/include/nodes/LagrangianPoint.hpp>
#include <LbmLib/include/Direction.hpp>
#include <vector>
#include <string>
#include <map>

namespace LbmLib {
namespace geometry {
class Connection;
}
namespace solver {
class BoundaryAbstractSolver;
}

namespace nodes {
class PhysicalNode;

/**
 * @brief class representing a boundary node
 */
class BoundaryNode : public LagrangianPoint {
 public:
    /**
     * @brief BoundaryNode Constructor for a boundary node
     * @param x The x pos
     * @param y The y pos
     * @param connectionType The connection of the solvers
     * @param domainId the domain id this node belongs to
     */
    explicit BoundaryNode(
            double x,
            double y,
            const std::map<std::string,
                    std::vector<std::string> >& connectionType,
            unsigned int domainId);
    /**
     * @brief ~BoundaryNode Destructor
     */
    ~BoundaryNode();

    /**
     * @brief setPhysicalNeighbours Sets the corresponding Physical neighbours of this node
     * @param physicalNode The physical Neighbour
     * @param dir The direction of the physical neighbour
     */
    void setPhysicalNeighbours(
            PhysicalNode* const physicalNode,
            const Direction& dir);

    /**
     * @brief getFluidNeighbour Getter for the Physical neighbour
     * @return A Pointer to the Physical neighbour
     */
    PhysicalNode* getPhysicalNeighbour() const;

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
     * @brief getBoundarySolver Getter method for the boundary Solver
     * @param name The name of the Boundary solver
     * @return The Boundary solver at this node
     */
    solver::BoundaryAbstractSolver& getBoundarySolver(const std::string& name);

    /**
     * @brief getBoundarySolvers Getter method for all Boundary Solvers
     * @return All Boundary Solvers
     */
    std::map<std::string, solver::BoundaryAbstractSolver*>& getBoundarySolvers();

    /**
     * @brief getDirectionToNeighbour returns the direction to the next neighbour
     * @return the direction to the next neighbour
     **/
    Direction getDirectionToNeighbour() const;

    /**
     * @brief getDomainIdentifier Getter for the domain Identifier
     * @return the domainIdentifier of this boundary Node
     */
    unsigned int getDomainIdentifier() const;

 private:
    /**
     * @brief directionToPhysicalNeighbour_ The direction where the next neighbour sits
     **/
    Direction directionToPhysicalNeighbour_;
    /**
       *@brief domainId_ The domain this boundary node belongs to
     */
    const unsigned int domainId_;

    /**
     * @brief physicalNeighbour_ Pointer to the physical Neighbour of this node
     */
    PhysicalNode* physicalNeighbour_;
    /**
     * @brief boundarySolvers_ The list of the boundary solvers
     */
    std::map<std::string, solver::BoundaryAbstractSolver*> boundarySolvers_;
    /**
     * @brief addBoundarySolver Adds a Boundary Solver to all boundary Nodes
     * @param name The name of the Boundary Solver
     */
    void addBoundarySolver(const std::string& name);
};
}   // end namespace
}  // end namespace
#endif  // INTERSECTIONPOINT_HPP
