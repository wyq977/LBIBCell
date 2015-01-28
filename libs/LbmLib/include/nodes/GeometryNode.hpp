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
#ifndef POLYGONPOINT_HPP
#define POLYGONPOINT_HPP

#include <LbmLib/include/nodes/LagrangianPoint.hpp>
#include <LbmLib/include/geometry/Connection.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <UtilLib/include/Log.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <array>
#include <iostream>
#include <ostream>
#include <string>

namespace LbmLib {
namespace geometry {
class Connection;
}
}

namespace LbmLib {
namespace nodes {
class PhysicalNode;

/**
 * @brief class representing a geometry node
 */
class GeometryNode : public LagrangianPoint {
 public:
    /**
     * @brief GeometryNode constructs a Geometry node
     * @param x the x pos
     * @param y the y pos
     * @param id the id of this node
     */
    GeometryNode(
            double x,
            double y,
            unsigned int id);
    /**
     * @brief ~GeometryNode Destructor
     */
    ~GeometryNode();


    /**
     * @brief writes the node to the stream
     * @param stream the output stream
     */
    void writeNode(std::ostream* stream);

    /**
     * @brief movePoint Moves the geomety node exactly one timestep
     */
    void move();

    /**
     * @brief addPhysicalNode Adds a physical node at a certain position
     * @param physicalNode A Pointer to a physical node
     * @param pos The internal position of the physical node
     */
    void addPhysicalNode(
            PhysicalNode* const physicalNode,
            unsigned int pos);

    /**
     * @brief getType The type of a node class
     * @return Returns the class Name of the point
     */
    virtual std::string getType() const;

    /**
     * @brief setPos Set the position new
     * override to make sure that perturbation occures
     * @param x The new x position
     * @param y The new y position
     */
    virtual void setPos(
            double x,
            double y);

    /**
     * @brief collectVelocity collects the velocities from the fluid nodes to the geometry Nodes
     */
    void collectVelocity();

    /**
     * @brief distributes the Force to the fluid nodes
     */
    void distributeForce();

    /**
     * @brief isUpdateNeeded Calculates if the connections of this points need to be updates
     * @return True if the point needs to redefine his connection
     */
    bool isUpdateNeeded() const;

    /**
     * @brief set the force of this node
     *
     * @param f the new force f on this node
     */
    void setForce(Field<double> f);

    /**
     * @brief adds the force to the force of this node
     *
     * @param f the force added
     */
    void addForce(Field<double> f);

    /**
     * @brief getter for the node id
     *
     * @return the node id of this node
     */
    unsigned int getId() const;

    /**
     * @brief setter for the Kth *Connection*
     * @param newconnection a pointer to the *Connection*
     */
    template<std::size_t K>
    void setConnection(const std::shared_ptr<LbmLib::geometry::Connection> newconnection)
    {
        assert(K==0 || K==1);
        if (this->connections_[K] != nullptr) {
            if (newconnection != nullptr) {
                std::stringstream message;
                message << std::setprecision(12) <<
                           "GeometryNode::setConnection<" << K << ">() on point ID= "<< this->getId() <<
                           " (" << this->getXPos() << "," << this->getYPos() << "): overwrite non-nullptr entry (this should not happen)";
                lbm_fail(message.str().c_str());
            }
        }
        this->connections_[K] = newconnection;
    }

    /**
     * @brief getter for the Kth *Connection*
     * @return a pointer to the *Connection*
     */
    template<std::size_t K>
    const std::shared_ptr<LbmLib::geometry::Connection>  getConnection() const
    {
        if (this->connections_[K] == nullptr) {
            std::stringstream message;
            message << std::setprecision(12) <<
                       "GeometryNode::getConnection<" << K << ">() on point ID= "<< this->getId() <<
                       " (" << this->getXPos() << "," << this->getYPos() << ")" <<
                       " at time " << Parameters.getCurrentIteration() <<
                       ": nullptr found (this should not happen)";
            LOG(UtilLib::logINFO) << message.str().c_str();
        }
        assert(K==0 || K==1);
        assert(this->connections_[K] != nullptr);
        return this->connections_[K];
    }

    /**
     * @brief get the domainID of the (max two) adjacent *Connection*s
     * @return The domainID
     */
    unsigned int getDomainIdOfAdjacentConnections(void) const;



 private:
    /**
     * @brief oldPoisition_ Stores the old positions
     */
    Field<double> oldPosition_;
    /**
     * @brief the force on this geometry node
     */
    Field<double> force_;
    /**
     * @brief perturbatePosition Makes sure that the Geometry point is not on a possible boundary position by moving it
     */
    void perturbatePosition();

    /**
     * @brief the id of this node
     */
    const unsigned int id_;

    /**
     * @brief neighbourPhysicalNodes_ Stores the 16 next physical nodes.
     */
    std::array<PhysicalNode*,
            16> neighbourPhysicalNodes_;

    /**
     * @brief connections_ Stores the (max two) connecting *Connection*s.
     * The index 0 is for the 'incoming' *Connection*, and index 1 for the 'outgoing' *Connection*.
     */
    std::array<std::shared_ptr<LbmLib::geometry::Connection>,
            2> connections_;


};
}   // end namespace
}  // end namespace

#endif  // POLYGONPOINT_HPP
