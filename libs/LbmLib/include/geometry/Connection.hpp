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
#ifndef GEOMETRY_CONNECTION_HPP
#define GEOMETRY_CONNECTION_HPP
#include <LbmLib/include/Direction.hpp>
#include <LbmLib/include/Field.hpp>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <unordered_set>

namespace LbmLib {
namespace nodes {
class GeometryNode;
class PhysicalNode;
class BoundaryNode;
class LagrangianPoint;
}

namespace geometry {
/**
 * @brief All PhysicalNodes on the left of the connection are defined as inside.
 */
class Connection {
 public:
    /**
     * @brief Connection A simple connection connecting two geometry nodes.
     * @param p1 GeometryNode 1; the first node defining this connection.
     * @param p2 GeometryNode 2; the second node defining this connection.
     * @param connectionType The connection type of the boundary solvers
     *      (first string: name of the BoundarySolver; second string: name of the affected CDESolver).
     * @param domainId The ID of the domain this connection sourrounds.
     */
    explicit Connection(
            std::shared_ptr<nodes::GeometryNode> const p1,
            std::shared_ptr<nodes::GeometryNode> const p2,
            const std::map<std::string,
                    std::vector<std::string> > connectionType,
            unsigned int domainId);
    /**
     * @brief ~Connection non-virutal Destructor
     */
    ~Connection();

    /**
     * @brief writes the connection ot the stream
     * @param stream the output stream
     */
    void writeConnection(std::ofstream* const stream);

    /**
     * @brief generateBoundaryNodes Calculates all possible boundary points defined by this connection, generates and connects them accordingly
     * @param physicalNode All physical nodes
     * @param boundaryNodes All boundary nodes found so far
     */
    void generateBoundaryNodes(
            const std::vector<std::vector<nodes::PhysicalNode*> >& physicalNode,
            std::unordered_set<nodes::BoundaryNode*>& boundaryNodes);

    /**
     * @brief getGeometryNodes Getter method for the geometry nodes defining this connection
     * @return The geometry nodes defining this connection
     */
    std::pair<std::shared_ptr<nodes::GeometryNode> const,
            std::shared_ptr<nodes::GeometryNode> const> getGeometryNodes()
    const;

    /**
     * @brief perturbConnection makes sure that the connection has a slope not near to 0 or infinitive
     */
    void perturbConnection();

    /**
     * @brief Compute the length of the connection.
     * @return The length.
     */
    double getLength() const;

    /**
     * @brief Getter for the domainID on the left side of this connection.
     * @return The domainID on the left side of this connection.
     */
    unsigned int getDomainIdentifier() const;

    /**
     * @brief Getter for the boundary condition descriptor.
     * @return The boundary condition descriptor.
     */
    const std::map<std::string, std::vector<std::string> > getBoundaryConditionDescriptor() const;

    /**
     * @brief Setter for the domainIdentifier_
     * @param domainIdentifier
     * @attention This method is casting away constness of domainIdentifier_!
     */
    void setDomainIdentifier(const unsigned int domainIdentifier);

 private:
    /**
     * @brief insertBoundaryNode Inserts a node
     * @param newBN The new boundary node
     * @param physicalNode The neighbouring physical node
     * @param d The direction of the physical neighbour
     * @param boundaryNodes All boundary nodes found so far
     */
    void insertBoundaryNode(
            nodes::BoundaryNode* const newBN,
            nodes::PhysicalNode* const physicalNode,
            const Direction& d,
            std::unordered_set<nodes::BoundaryNode*>& boundaryNodes);

    /**
     * @brief Linear interpolation of the velocity of the GeometryNodes at the position of the boundary node.
     * @param bnode The boundary node.
     * @return The velocity of the boundary node.
     */
    Field<double> calculateVelocity(const nodes::BoundaryNode& bnode);

    /**
     * @brief points_ Storage of the points defining this connection
     */
    const std::pair<std::shared_ptr<nodes::GeometryNode> const,
            std::shared_ptr<nodes::GeometryNode> const> points_;
    /**
     * @brief connectionType_ The connection of the boundarySolvers to the CDESolvers {boundarysolver,cdesolver}
     */
    const std::map<std::string, std::vector<std::string> > connectionType_;
    /**
     * @brief domainIdentifier_ the domain identifer of this connection
     */
    unsigned int domainIdentifier_;
};
}   // end namespace
}  // end namespace
#endif  // GEOMETRY_CONNECTION_HPP
