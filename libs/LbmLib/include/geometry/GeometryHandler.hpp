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
#ifndef GEOMETRY_GEOMETRYHANDLER_HPP
#define GEOMETRY_GEOMETRYHANDLER_HPP

#include <LbmLib/include/geometry/Connection.hpp>
#include <LbmLib/include/geometry/Geometry.hpp>
#include <iostream>
#include <list>
#include <memory>
#include <ostream>
#include <string>
#include <vector>
#include <unordered_set>

namespace LbmLib {
namespace nodes {
class PhysicalNode;
class GeometryNode;
class BoundaryNode;
}

namespace geometry {
/**
 * @brief class responsible for generating the internal geometry representation
 */
class GeometryHandler {
 public:
    /**
     * @brief GeometryHandler Constructs the simulation grid
     * @param geometry The geometry
     */
    explicit GeometryHandler(const Geometry& geometry);


    /**
     * @brief moveLattcie Updates the Lattice after the Geometric Points have been moved
     */
    void moveLattice();

    /**
     * @brief If a *Connection* is too long, a GeometryNode is added and linked.
     * @return The number of remeshed *Connection*s
     */
    unsigned int remeshBoundary();

    /**
     * @brief If *Connection*s are too short, a GeometryNode is remove.
     */
    void coarsenBoundary();

    /**
     * @brief ~GeometryHandler Destroyes all nodes.
     */
    ~GeometryHandler();

    /**
     * @brief getPhysicalNodes Getter method for the physical node grid
     * @return The physical node grid
     */
    const std::vector<std::vector<nodes::PhysicalNode*> >& getPhysicalNodes()
    const {
        return physicalGrid_;
    }

    /**
     * @brief getter for the geometry
     * @return a reference to the geometry
     */
    const Geometry& getGeometry() const {
        return geometry_;
    }

    /**
     * @brief Getter for the Boundary nodes
     * @return a list of all boundary nodes
     */
    const std::unordered_set<nodes::BoundaryNode*>& getBoundaryNodes() const {
        return boundaryNodes_;
    }

    /**
     * @brief Compute the areas of the domains by using the domainIdentifiers.
     * @return map with domainIdentifier as key and it's area
     */
    const std::map<unsigned int,double> computeAreas() const;

    /**
     * @brief Compute the accumulated concentrations of species *name* in all domains
     * @param name The species to look at.
     * @return map with domainIdentifier as key and it's accumulated concentration of species *name*
     */
    std::map<unsigned int,double> computeAccumulatedDomainConcentrations(const std::string& name) const;

    /**
     * @brief Cure the Lattice: update *BoundaryNode*s, DomainIdentifier, and IB connections.
     */
    void cureLattice();

    /**
     * @brief Add a new GeometryNode
     * @param xpos The x position.
     * @param ypos The y position.
     * @return the GeometryNode ID
     */
    unsigned int createGeometryNode(const double xpos,const double ypos);

    /**
     * @brief Create a new connection
     * @param p1 The first GeometryNode.
     * @param p2 The second GeometryNode.
     * @param boundaryconditiondescriptor The boundary condition descriptor.
     * @param domainidentifier The domain identifier.
     */
    void createConnection(std::shared_ptr<nodes::GeometryNode> const p1,
                          std::shared_ptr<nodes::GeometryNode> const p2,
                          const std::map<std::string, std::vector<std::string> > boundaryconditiondescriptor,
                          const unsigned int domainidentifier);

    /**
     * @brief Erases the Connection.
     * @param toErase The Connection to be erased.
     */
    void eraseConnection(std::shared_ptr<Connection> toErase);

    /**
     * @brief returnGeometryNode
     * @param nodeID
     * @return pointer to the GeometryNode
     */
    const std::shared_ptr<nodes::GeometryNode> returnGeometryNode(const unsigned int nodeID) const;

    /**
     * @brief Check lattice integrity
     */
    void checkLatticeIntegrity();

    /**
     * @brief checkBoundaryNodeIntegrity Checks the integrity of the boundary node pairs.
     */
    void checkBoundaryNodeIntegrity();

    /**
     * @brief update the celltypes of all *PhysicalNode*s with domainid
     * @param celltrackermap The map storing {domainID,cellType}
     */
    void copyCellTypeToPhysicalNodes(std::map<unsigned int,unsigned int> &celltrackermap);

    /**
     * @brief update the celltypes of only *PhysicalNode*s with domainidentifier to celltype
     * @param domainidentifier The affected domain.
     * @param celltype The celltype to be set.
     */
    void copyCellTypeToPhysicalNodes(unsigned int domainidentifier,unsigned int celltype);

    /**
     * @brief Returns a reference to the cellTypeTrackerMap.
     * @return The reference to the cellTypeTrackerMap.
     */
    std::map<unsigned int,unsigned int>&
    getCellTypeTrackerMap(void);

    /**
     * @brief checkGeometryIntegrity
     * @return true if integrity is given. fails otherwise.
     */
    bool checkGeometryIntegrity(void) const;


 private:
    /**
       *@brief perturbConnections perturb all Connections until they are all perturbed
     */
    void perturbConnections();

    /**
     * @brief generateBoundaryNodes Creates all boundary nodes from the connections
     */
    void generateBoundaryNodes();

    /**
     * @brief generatePhysicalGrid Generates the physical nodes
     */
    void generatePhysicalGrid();

    /**
     * @brief makes the internal connections of the physical grid
     */
    void makePhysicalGridConnections();

    /**
     * @brief makes the boundary connection of the physical grid
     */
    void makePeriodicBoundary();

    /**
     * @brief Connect all GeometryNode with 16 neighboring PhysicalNode
     * @param pt The GeometryNode which should be reconnected
     */
    void connectGeometryNodesToPhysicalNodes(std::shared_ptr<nodes::GeometryNode> pt);

    /**
     * @brief initGeometryNodes Initialise all Geometry nodes.
     */
    void connectGeometryNodesToPhysicalNodes();

    /**
     * @brief GeometryHandler::updateDomainIdentifier To force reinitialization of CDE Solvers.
     */
    void updateDomainIdentifier();

    /**
       *@brief updateAllDomainIdentifiers initialises the physical nodes with the correct domain identifier.
     */
    void updateAllDomainIdentifiers();

    /**
     * @brief cellTypeTrackerMap_ Stores the cell type for each cell {domainID, cellType}
     */
    std::map<unsigned int,unsigned int> cellTypeTrackerMap_;

    /**
     * @brief physicalGrid_ Storage of the physical nodes. First dimension is y, second dimension is x.
     */
    std::vector<std::vector<nodes::PhysicalNode*> > physicalGrid_;
    /**
     * @brief boundaryNodes_ Storage of the boundary nodes
     */
    std::unordered_set<nodes::BoundaryNode*> boundaryNodes_;

    const Geometry& geometry_;  ///< reference to the geometry
};
}   // end namespace
}  // end namespace

#endif  // GEOMETRY_GEOMETRYHANDLER_HPP
