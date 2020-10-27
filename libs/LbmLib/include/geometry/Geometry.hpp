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
#ifndef GEOMETRY_GEOMETRY_HPP
#define GEOMETRY_GEOMETRY_HPP

#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/Constants.hpp>
#include <UtilLib/include/geometry/FastNeighborList.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/index/rtree.hpp>

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(LbmLib::nodes::GeometryNode,
                                         double,
                                         cs::cartesian,
                                         LbmLib::nodes::LagrangianPoint::getXPos,
                                         LbmLib::nodes::LagrangianPoint::getYPos,
                                         LbmLib::nodes::LagrangianPoint::setXPos,
                                         LbmLib::nodes::LagrangianPoint::setYPos)

namespace boost {
namespace geometry {
namespace index {

//specializing indexable to store shared_ptr in rtree:
template <typename Value>
struct indexable <std::shared_ptr<Value> >
{
    typedef Value const& result_type;
    result_type operator() (std::shared_ptr<Value> const& v) const { return *v; }
};

} // end namespace index
} // end namespace geometry
} // end namespace boost

namespace LbmLib {
namespace geometry {

class Connection;

/**
 * @brief class representing the external geometry
 */
class Geometry {
 public:
    /**
     * @brief Geometry constructs the geometry of the simulation
     * @param filename the filename where the geometry is specified
     */
    explicit Geometry(const std::string& filename);

    /**
     * @brief ~Geometry Plain.
     */
    ~Geometry();

    /**
     * @brief writeGeometry Writes the geometry to the file
     * @param fileName the filename
     */
    void writeGeometry(const std::string& fileName) const;

    /**
     * @brief Getter for the geometry nodes
     * @return a list of all geometry nodes
     */
    const std::map<unsigned int,
            std::shared_ptr<nodes::GeometryNode> >& getGeometryNodes() const {
        return geometryNodes_;
    }

    /**
     * @brief getGeometryNodesWithinRadius Range query
     * @param x The x coordinate.
     * @param y The ycoordinate.
     * @param radius The radius to search within.
     * @return A map containing the *GeometryNode*s
     */
    const std::vector<std::shared_ptr<nodes::GeometryNode> > getGeometryNodesWithinRadius(const double x,
                                                                                          const double y,
                                                                                          const double radius) const;



    /**
     * @brief getGeometryNodesWithinRadiusWithAvoidance Range query, but only nodes with domainID different from avoidDomainID
     * @param x The x coordinate.
     * @param y The ycoordinate.
     * @param avoidDomainID The domainID to be avoided when returning *GeometryNode*s.
     * @param radius The radius to search within.
     * @return A vector containing the *GeometryNode*s
     */
    const std::vector<std::shared_ptr<nodes::GeometryNode> > getGeometryNodesWithinRadiusWithAvoidance(const double x,
                                                                                                       const double y,
                                                                                                       const double radius,
                                                                                                       const unsigned int avoidDomainID) const;
    /**
     * @brief getGeometryNodesWithinRadiusWithAvoidanceClosest Return closest *GeometryNode*, but only nodes with domainID different from avoidDomainID
     * @param x The x coordinate.
     * @param y The ycoordinate.
     * @param avoidDomainID The domainID to be avoided when returning *GeometryNode*s.
     * @param radius The radius to search within.
     * @return The closest different *GeometryNode*, or nullptr if none.
     */
    std::shared_ptr<nodes::GeometryNode> getGeometryNodesWithinRadiusWithAvoidanceClosest(const double x,
                                                     const double y,
                                                     const double radius,
                                                     const unsigned int avoidDomainID) const;

    /**
     * @brief getConnections Getter for connections
     * @return The connections
     */
    const std::vector<std::shared_ptr<Connection> >& getConnections() const;

    /**
     * @brief addGeometryNode Add a GeometryNode. The NodeID is bumped automatically.
     * @param x The x coordinate.
     * @param y The y coordinate.
     * @return The ID under which the GeometryNode is registered.
     */
    unsigned int addGeometryNode(const double x,const double y);

    /**
     * @brief removeGeometryNode Removes the GeometryNode with nodeid. One of the connections is removed, the other is connected accordingly.
     * @param nodeid The GeometryNode identifier.
     * @return 1 if successful, 0 otherwise.
     */
    unsigned int removeGeometryNode(const unsigned int nodeid);

    /**
     * @brief eraseConnection Erase the connection.
     * @param toDelete The pointer to the Connection which shall be deleted.
     */
    void eraseConnection(std::shared_ptr<Connection> toDelete);

    /**
     * @brief addConnection Add a Connection.
     * @param p1 The first GeometryNode.
     * @param p2 The second GeometryNode.
     * @param boundaryConditionDescriptor
     * @param domainIdentifier The domainIdentifier.
     */
    void addConnection(std::shared_ptr<nodes::GeometryNode> p1,
                       std::shared_ptr<nodes::GeometryNode> p2,
                       const std::map<std::string, std::vector<std::string> > boundaryConditionDescriptor,
                       const unsigned int domainIdentifier);

    /**
     * @brief Geometry::moveGeometryNodes moves the *GeometryNode*s according to the local velocity field
     */
    void moveGeometryNodes();

    /**
     * @brief checkGeometryIntegrity
     * @return true if integrity is given. fails otherwise.
     */
    bool checkGeometryIntegrity() const;

    /**
     * @brief invalidateRangeQueryDataStructure Sets the internal flag to false.
     */
    void invalidateRangeQueryDataStructure();

    /**
     * @brief reconstructRangeQueryDataStructure
     */
    void reconstructRangeQueryDataStructure() const;

    /**
     * @brief Returns a reference to the cellTypeTrackerMap.
     * @return The reference to the cellTypeTrackerMap.
     */
    std::map<unsigned int, unsigned int> getCellTypeTrackerMap(void) const;

    /**
     * @brief removeGeometryNodeWithoutReconnecting Removes the node from the fastneighborlist_ and geometryNodes_ datastructues
     * @param nodeid The ID of the node to be removed.
     */
    void removeGeometryNodeWithoutReconnecting(const unsigned int nodeid);
    
private:
    /**
     * @brief loads the geometry from a file
     * @param fileName the Filename.
     */
    void loadGeometryTXT(const std::string& fileName);

    /**
     * @brief loadGeometryVTK Loads the geometry from a .vtm format.
     * @param fileName The filename.
     */
    void loadGeometryVTK(const std::string& fileName);

    /**
     * @brief connections_ Stores the connections
     */
    std::vector<std::shared_ptr<Connection> > connections_;

    /**
     * @brief geometryNodes_ Storage of the geometry nodes.
     * The key value is the GeometryNodeID.
     */
    std::map<unsigned int,
            std::shared_ptr<nodes::GeometryNode> > geometryNodes_;

    /**
     * @brief fastneighborlist_ The cell list data structure for range queries.
     */
    mutable UtilLib::geometry::fastneighborlist< std::shared_ptr<nodes::GeometryNode> > fastneighborlist_;

    /**
     * @brief isValidRangeQueryDataStructure_ flag for the validity of the range query data structure
     */
    mutable bool isValidRangeQueryDataStructure_;

    /**
     * @brief cellTypeTrackerMap_ Stores the cell type for each cell {domainID, cellType}
     * @return The cellTypeTrackerMap_ by value.
     */
    std::map<unsigned int,unsigned int> cellTypeTrackerMap_;
};

}   // end namespace
}  // end namespace
#endif  // GEOMETRY_GEOMETRY_HPP
