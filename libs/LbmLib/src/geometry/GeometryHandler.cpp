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
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/nodes/BoundaryNode.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>

#include <LbmLib/include/Constants.hpp>
#include <LbmLib/include/Direction.hpp>

#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Log.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <cassert>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>
#include <utility>
#include <map>
#include <omp.h>

namespace LbmLib {
namespace geometry {
GeometryHandler::GeometryHandler(const Geometry& geometry)
    : geometry_(geometry) {
    this->generatePhysicalGrid(); //generate PhysicalNodes and connections between them
    this->cureLattice(); // set up IB links, *BoundaryNode*s and domainIdentifiers
    this->cellTypeTrackerMap_ = geometry.getCellTypeTrackerMap();
}

void GeometryHandler::updateDomainIdentifier() {
#pragma omp parallel for schedule(dynamic)
    for (size_t y = 0; y < physicalGrid_.size(); y++) {
        for (size_t x = 0; x < physicalGrid_[0].size(); x++) {
            assert(physicalGrid_[y][x] != nullptr);
            physicalGrid_[y][x]->updateDomainIdentifier();
        }
    }
}

void GeometryHandler::updateAllDomainIdentifiers()
{
#pragma omp parallel for schedule(dynamic)
    for (unsigned int y = 0; y < Parameters.getSizeY(); y++) { // loop y
        unsigned int currentDomainId =
            this->physicalGrid_[y][0]->getDomainIdentifier();
        if (currentDomainId!=0) {
            lbm_fail("domainID at the left boundary must be 0.");
        }
        for (unsigned int x = 1; x < Parameters.getSizeX(); x++) { // loop x
            if (this->physicalGrid_[y][x]->getBoundaryNeighbour(W) != nullptr ) { // look backward
                currentDomainId =
                    this->physicalGrid_[y][x]->getBoundaryNeighbour(W)->getDomainIdentifier();
            }
            this->physicalGrid_[y][x]->setDomainIdentifier(currentDomainId);
        }
    }
}

GeometryHandler::~GeometryHandler() {
    for (auto it : physicalGrid_) {
        for (auto i : it) {
            delete i;
            i = nullptr;
        }
    }

    for (auto b : boundaryNodes_) {
        delete b;
        b = nullptr;
    }
}

const std::map<unsigned int, double> GeometryHandler::computeAreas() const
{
    typedef boost::geometry::model::d2::point_xy<double> point;
    typedef boost::geometry::model::polygon< point,false,false > Polygon;
    std::map<unsigned int,std::vector<std::shared_ptr<LbmLib::geometry::Connection> > > celldefinition;
    std::map<unsigned int, Polygon> polygonmap;
    std::shared_ptr<LbmLib::geometry::Connection> startC = nullptr;
    std::shared_ptr<LbmLib::geometry::Connection> tempC = nullptr;
    std::map<unsigned int, double> arealist;

    // step 1: create a cell definition map:
    for (auto it : this->getGeometry().getConnections()) {
        celldefinition[(*it).getDomainIdentifier()].push_back(it);
    }

    // step 2: create a polygon map:
    for (auto it : celldefinition) {
        startC = it.second[0];
        tempC = startC;
        do {
            boost::geometry::append(polygonmap[it.first], boost::geometry::make<point>(
                                        tempC->getGeometryNodes().second->getXPos(),
                                        tempC->getGeometryNodes().second->getYPos()));
            tempC = tempC->getGeometryNodes().second->getConnection<1>();
        } while(tempC != startC);
    }

    // step 3: compute areas
    for (auto it : polygonmap) {
        arealist[it.first] = boost::geometry::area(it.second);
    }

    return arealist;
}

std::map<unsigned int,double> GeometryHandler::computeAccumulatedDomainConcentrations(const std::string &name) const
{
    std::map<unsigned int, double> accumulatedcellconcentration;

    for (unsigned int y = 0; y < Parameters.getSizeY(); y++) { // loop y
        for (unsigned int x = 1; x < Parameters.getSizeX(); x++) { // loop x
            accumulatedcellconcentration[this->physicalGrid_[y][x]->getDomainIdentifier()] +=
                    this->physicalGrid_[y][x]->getCDESolverSlow(name).getC();;
        }
    }
    return accumulatedcellconcentration;
}

void GeometryHandler::cureLattice()
{
    // step 1: perturb connections
    this->perturbConnections();

    // step 2: reconnect all GeometryNodes to PhysicalNodes:
    this->connectGeometryNodesToPhysicalNodes();

    // step 3: delete all BoundaryNodes:
    for (auto b : this->boundaryNodes_) {
        b->getPhysicalNeighbour()->resetBoundaryNodes();
        delete b;
        b = nullptr;
    }
    this->boundaryNodes_.clear();

    // step 4: re-generate BoundaryNodes:
    this->generateBoundaryNodes();

    // step 5: update all domain identifiers of the *PhysicalNode*s
    this->updateAllDomainIdentifiers();
}

unsigned int GeometryHandler::createGeometryNode(const double xpos, const double ypos)
{
    return const_cast<geometry::Geometry&>(this->geometry_).addGeometryNode(xpos,ypos);
}

void GeometryHandler::createConnection(const std::shared_ptr<nodes::GeometryNode> p1,
                                       const std::shared_ptr<nodes::GeometryNode> p2,
                                       const std::map<std::string, std::vector<std::string> > boundaryconditiondescriptor,
                                       const unsigned int domainidentifier)
{
    const_cast<geometry::Geometry&>(this->geometry_).addConnection(p1,
                                                                   p2,
                                                                   boundaryconditiondescriptor,
                                                                   domainidentifier);
}

void GeometryHandler::eraseConnection(std::shared_ptr<Connection> toErase)
{
    const_cast<geometry::Geometry&>(this->geometry_).eraseConnection(toErase);
}

const std::shared_ptr<nodes::GeometryNode> GeometryHandler::returnGeometryNode(const unsigned int nodeID) const
{
    return this->geometry_.getGeometryNodes().at(nodeID);
}

void GeometryHandler::checkLatticeIntegrity()
{
    unsigned int myID;
    std::array<Direction, 4> dir {{N,S,E,W}};

    for (unsigned int y = 0; y < Parameters.getSizeY(); y++) { // loop y
        for (unsigned int x = 0; x < Parameters.getSizeX(); x++) { // loop x
            myID = this->physicalGrid_[y][x]->getDomainIdentifier();

            // check if domainID's are consistent if neighbor is PhysicalNode:
            for (auto d : dir) {
                if (this->physicalGrid_[y][x]->getBoundaryNeighbour(d) == nullptr) {
                    if (myID != this->physicalGrid_[y][x]->getPhysicalNeighbour(d)->getDomainIdentifier()) {
                        std::stringstream message;
                        message << "Grid integrity not given (domainID jump to PhysicalNode neighbor): ";
                        message << "time=" << Parameters.getCurrentIteration() << "; ";
                        message << "position=(" << x << "," << y << "); ";
                        message << "my domainID=" << myID << "; ";
                        message << "neighbor domainID=" << this->physicalGrid_[y][x]->getPhysicalNeighbour(d)->getDomainIdentifier() << "; ";
                        message << "direction=" << d;
                        LOG(UtilLib::logINFO) << message.str().c_str();
                        lbm_fail(message.str().c_str());
                    }
                }
            }

            // check if domainID's are consistent if neighbor is BoundaryNode:
            for (auto d : dir) {
                if (this->physicalGrid_[y][x]->getBoundaryNeighbour(d) != nullptr) {
                    if (myID != this->physicalGrid_[y][x]->getBoundaryNeighbour(d)->getDomainIdentifier()) {
                        std::stringstream message;
                        message << "Grid integrity not given (domainID jump to BoundaryNode neighbor): ";
                        message << "time=" << Parameters.getCurrentIteration() << "; ";
                        message << "position=(" << x << "," << y << "); ";
                        message << "my domainID=" << myID << "; ";
                        message << "neighbor domainID=" << this->physicalGrid_[y][x]->getBoundaryNeighbour(d)->getDomainIdentifier() << "; ";
                        message << "direction=" << d;
                        LOG(UtilLib::logINFO) << message.str().c_str();
                        lbm_fail(message.str().c_str());
                    }
                }
            }
        }
    }
}

void GeometryHandler::checkBoundaryNodeIntegrity()
{
    std::array<Direction, 4> dir {{N,S,E,W}};

    for (unsigned int y = 0; y < Parameters.getSizeY(); y++) { // loop y
        for (unsigned int x = 0; x < Parameters.getSizeX(); x++) { // loop x
            for (auto d : dir) {
                if (this->physicalGrid_[y][x]->getBoundaryNeighbour(d) != nullptr) {
                    if (this->physicalGrid_[y][x]->getPhysicalNeighbour(d)->getBoundaryNeighbour(getInverseDirection(d)) == nullptr) {
                        std::stringstream message;
                        message << "BoundaryNode integrity not given: ";
                        message << "time=" << Parameters.getCurrentIteration() << "; ";
                        message << "PhysicalNode (" << x << "," << y << "); ";
                        message << "direction " << d << "; ";
                        LOG(UtilLib::logINFO) << message.str().c_str();
                        lbm_fail(message.str().c_str());
                    }
                }
            }
        }
    }
}

void GeometryHandler::copyCellTypeToPhysicalNodes(std::map<unsigned int,unsigned int> &celltrackermap)
{
    unsigned int id;

#pragma omp parallel for schedule(dynamic) private(id)
    for (size_t y = 0; y < physicalGrid_.size(); y++) {
        for (size_t x = 0; x < physicalGrid_[0].size(); x++) {
            assert(physicalGrid_[y][x] != nullptr);
            id = this->physicalGrid_[y][x]->getDomainIdentifier();

            //check if id is a valid key in celltrackermap:
            if (celltrackermap.find(id) == celltrackermap.end() )
            {
                // key 2 doesn't exist
#pragma omp critical
                {
                    celltrackermap[id] = 1;
                    std::stringstream message;
                    message << "GeometryHandler::copyCellTypeToPhysicalNodes(): Could not find domainID="
                            << id <<"; added it and set to cellType=1. " ;

                    message << "The celltrackermap reads: [domainID/cellType]: ";
                    for (auto it : celltrackermap) {
                       message << "[" << it.first << "/" << it.second << "] ";
                    }
                    LOG(UtilLib::logINFO) << message.str().c_str();
                }
            }

            this->physicalGrid_[y][x]->setCellType(
                        celltrackermap[id]
                        );
        }
    }
}

void GeometryHandler::copyCellTypeToPhysicalNodes(unsigned int domainidentifier, unsigned int celltype)
{
#pragma omp parallel for schedule(dynamic)
    for (size_t y = 0; y < physicalGrid_.size(); y++) {
        for (size_t x = 0; x < physicalGrid_[0].size(); x++) {
            assert(physicalGrid_[y][x] != nullptr);
            if (this->physicalGrid_[y][x]->getDomainIdentifier() == domainidentifier) {
                this->physicalGrid_[y][x]->setCellType(celltype);
            }
        }
    }
}

std::map<unsigned int,unsigned int>& GeometryHandler::getCellTypeTrackerMap()
{
    return this->cellTypeTrackerMap_;
}

bool GeometryHandler::checkGeometryIntegrity() const
{
    return this->geometry_.checkGeometryIntegrity();
}

bool GeometryHandler::removeCell(solver::ForceSolver& forcesolver, unsigned int domainidentifier)
{
    auto tempnodesmap = this->getGeometry().getGeometryNodes();

    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > vecConnectionsToDelete;


    for (auto i : tempnodesmap) { // loop all GeometryNodes
        if (i.second->getDomainIdOfAdjacentConnections() == domainidentifier) {
            const_cast<geometry::Geometry&>(this->geometry_).removeGeometryNodeWithoutReconnecting(i.second->getId());

            // storing connections to delete. will be deleted at the end of this function.
            vecConnectionsToDelete.push_back(i.second->getConnection<0>());

            // also delete all forces which are asssociated with this GeometryNode:
            forcesolver.deleteForcesAssociatedWithNodeOn(i.second->getId());
            assert( forcesolver.checkIfNodeAssociationExists(i.second->getId()) );
        }
    }

    // get rid of duplicates:
    std::sort( vecConnectionsToDelete.begin(), vecConnectionsToDelete.end() );
    vecConnectionsToDelete.erase( std::unique( vecConnectionsToDelete.begin(), vecConnectionsToDelete.end() ), vecConnectionsToDelete.end() );

    //now delete all the unnecessary connections:
    for (unsigned int i=0; i<vecConnectionsToDelete.size(); ++i) {
        const_cast<geometry::Geometry&>(this->geometry_).eraseConnection(vecConnectionsToDelete[i]);
    }

    return 0;
}

void GeometryHandler::perturbConnections() {
    std::map<unsigned int,std::shared_ptr<LbmLib::geometry::Connection> > celldefinitionmap;
    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > celldefinitionvector;
    std::shared_ptr<LbmLib::geometry::Connection> startC = nullptr;
    std::shared_ptr<LbmLib::geometry::Connection> tempC = nullptr;

    // step 1: create a cell definition map with only one connection per domainIdentifier:
    for (auto it : this->getGeometry().getConnections()) {
        if (celldefinitionmap.find((*it).getDomainIdentifier()) == celldefinitionmap.end()) { // check if not existing yet
            celldefinitionmap[(*it).getDomainIdentifier()] = it; // if not yet existing, add
        }
    }

    // step 2: cast everything into a vector for openmp looping:
    for (auto it : celldefinitionmap) {
        celldefinitionvector.push_back(it.second);
    }

    //step 3: loop all cells and perturb. each cell is perturbed sequentially.
#pragma omp parallel for schedule(dynamic) private(startC,tempC)
    for (size_t it=0;
         it<celldefinitionvector.size();
         ++it) {
        startC = celldefinitionvector[it];
        tempC = startC;
        do {
            tempC->perturbConnection();
            tempC = tempC->getGeometryNodes().second->getConnection<1>();
        } while(tempC != startC);
    }
}

void GeometryHandler::moveLattice() {
    // step 1: move the GeometryNodes:
    const_cast<geometry::Geometry&>(this->geometry_).moveGeometryNodes();
    this->perturbConnections();

    // step 2: delete all BoundaryNodes:
    for (auto b : this->boundaryNodes_) {
        b->getPhysicalNeighbour()->resetBoundaryNodes();
        delete b;
        b = nullptr;
    }
    this->boundaryNodes_.clear();

    // step 3: re-generate BoundaryNodes:
    this->generateBoundaryNodes();

    // step 4: reconnect GeometryNodes to PhysicalNodes:
    this->connectGeometryNodesToPhysicalNodes();

    // step 5: calls updateDomainIdentifier() of PhysicalNode:
    this->updateDomainIdentifier(); // force CDE solver reinit
    this->updateAllDomainIdentifiers();

    // step 6: update cell types
    this->copyCellTypeToPhysicalNodes(this->cellTypeTrackerMap_);
}

unsigned int GeometryHandler::remeshBoundary() {
    // copy the connection vector in order not to interfer with modifications on the original:
    std::vector<std::shared_ptr<Connection> > concopy(this->geometry_.getConnections());
    unsigned int counter = 0;

    for (auto c = concopy.begin();
         c != concopy.end();
         c++) {
        if ((*c)->getLength()>MAXLENGTH) {
            // step 1: add the new GeometryNode
            const double x1 = (*c)->getGeometryNodes().first->getXPos();
            const double y1 = (*c)->getGeometryNodes().first->getYPos();
            const double x2 = (*c)->getGeometryNodes().second->getXPos();
            const double y2 = (*c)->getGeometryNodes().second->getYPos();
            const unsigned int newnodeID =
                    this->createGeometryNode((x1+x2)/2,(y1+y2)/2);

            const std::shared_ptr<nodes::GeometryNode> newnode = // get pointer to the newly created GeometryNode
                    this->returnGeometryNode(newnodeID);

            // step 2: get the signature of the old connection
            std::pair<std::shared_ptr<nodes::GeometryNode> const,
                    std::shared_ptr<nodes::GeometryNode> const>  adjacentnodes = (*c)->getGeometryNodes();
            std::map<std::string, std::vector<std::string> > descriptor = (*c)->getBoundaryConditionDescriptor();
            const unsigned int domainID = (*c)->getDomainIdentifier();

            // step 3: delete the old connection
            this->eraseConnection(*c);

            // step 4: add two new connections
            this->createConnection(adjacentnodes.first,
                                   newnode,
                                   descriptor,
                                   domainID);
            this->createConnection(newnode,
                                   adjacentnodes.second,
                                   descriptor,
                                   domainID);
            counter++;
        }
    }

    // step 5: reconnect GeometryNodes to PhysicalNodes:
    this->connectGeometryNodesToPhysicalNodes();

    return counter;
}

void GeometryHandler::coarsenBoundary()
{
    double length;
    Field<double> P1;
    Field<double> P2;

    // loop all *GeometryNode*s
    for (auto it = this->getGeometry().getGeometryNodes().begin();
         it!=this->getGeometry().getGeometryNodes().end();
         ) {

        // determine length without me
        P1 = (*it).second->getConnection<0>()->getGeometryNodes().first->getPos();
        P2 = (*it).second->getConnection<1>()->getGeometryNodes().second->getPos();
        length = std::sqrt( (P2.x-P1.x)*(P2.x-P1.x) + (P2.y-P1.y)*(P2.y-P1.y) );

        if (length < MAXLENGTH) {
            std::stringstream message;
            message << "GeometryHandler::coarsenBoundary(): "
                    << "removed GeometryNode=" << (*it).first <<"; "
                    << "length=" << length;
            const_cast<geometry::Geometry&>(this->getGeometry()).removeGeometryNode((*it++).first);
            LOG(UtilLib::logINFO) << message.str().c_str();
        }
        else {
            ++it;
        }
    }
}

void GeometryHandler::generatePhysicalGrid() {
    for (unsigned int y = 0; y < Parameters.getSizeY(); y++) {
        std::vector<nodes::PhysicalNode*> temp;
        for (unsigned int x = 0; x < Parameters.getSizeX(); x++) {
            temp.push_back(new nodes::PhysicalNode(x, y));
        }
        physicalGrid_.push_back(temp);
    }

    this->makePhysicalGridConnections();
    this->makePeriodicBoundary();
}

void GeometryHandler::makePhysicalGridConnections() {
    // make the connections.

    for (auto it : physicalGrid_) {
        for (auto i : it) {
            unsigned int x = i->getXPos();
            unsigned int y = i->getYPos();
            // not most left wall
            if (x != Parameters.getSizeX() - 1) {
                i->setPhysicalNeighbour(physicalGrid_[y][x + 1], E);
            }
            // not NE corner
            if ((x != Parameters.getSizeX() - 1) &&
                (y != Parameters.getSizeY() - 1) ) {
                i->setPhysicalNeighbour(physicalGrid_[y + 1][x + 1], NE);
            }
            // not top line
            if (y != Parameters.getSizeY() - 1) {
                i->setPhysicalNeighbour(physicalGrid_[y + 1][x], N);
            }
            // not NW corner
            if ((x != 0) && (y != Parameters.getSizeY() - 1)) {
                i->setPhysicalNeighbour(physicalGrid_[y + 1][x - 1], NW);
            }
            // not right wall
            if (x != 0) {
                i->setPhysicalNeighbour(physicalGrid_[y][x - 1], W);
            }
            // not SW corner
            if ((x != 0) && (y != 0)) {
                i->setPhysicalNeighbour(physicalGrid_[y - 1][x - 1], SW);
            }
            // not bottom line
            if (y != 0) {
                i->setPhysicalNeighbour(physicalGrid_[y - 1][x], S);
            }
            // not SE corner
            if ((x != Parameters.getSizeX() - 1) && (y != 0)) {
                i->setPhysicalNeighbour(physicalGrid_[y - 1][x + 1], SE);
            }
        }
    }
}

void GeometryHandler::makePeriodicBoundary() {
    // make the periodic boundary condition
    for (auto it : physicalGrid_) {
        for (auto i : it) {
            unsigned int x = i->getXPos();
            unsigned int y = i->getYPos();

            if (x == Parameters.getSizeX() - 1) {
                i->setPhysicalNeighbour(physicalGrid_[y][0], E);  // set east direction
                if (y != 0) {
                    i->setPhysicalNeighbour(physicalGrid_[y - 1][0], SE);  // set south east direction
                } else {
                    i->setPhysicalNeighbour(physicalGrid_[Parameters.getSizeY()
                                                          - 1][0], SE);  // its the SE edge
                }
                if (y != Parameters.getSizeY() - 1) {
                    i->setPhysicalNeighbour(physicalGrid_[y + 1][0], NE);  // set north east direction
                } else {
                    i->setPhysicalNeighbour(physicalGrid_[0][0], NE);   // its the NE edge
                }
            }

            if (y == Parameters.getSizeY() - 1) {
                i->setPhysicalNeighbour(physicalGrid_[0][x], N);  // set north direction
                if (x != Parameters.getSizeX() - 1) {
                    i->setPhysicalNeighbour(physicalGrid_[0][x + 1], NE);  // set north east direction
                }       // edge already setted
                if (x != 0) {
                    i->setPhysicalNeighbour(physicalGrid_[0][x - 1], NW);  // set north west direction
                }       // edge already setted
            }

            if (x == 0) {
                if (y != 0) {
                    i->setPhysicalNeighbour(physicalGrid_[y -
                                                          1][Parameters.getSizeX()
                                                             - 1], SW);   // set south west direction
                } else {
                    i->setPhysicalNeighbour(physicalGrid_[Parameters.getSizeY()
                                                          - 1][Parameters.
                                                                 getSizeX() - 1],
                            SW);  // its the SW edge
                }
                if (y != Parameters.getSizeY() - 1) {
                    i->setPhysicalNeighbour(physicalGrid_[y + 1][Parameters.
                                                                   getSizeX() -
                                                                 1], NW);  // set north west direction
                } else {
                    i->setPhysicalNeighbour(physicalGrid_[0][Parameters.
                                                               getSizeX() - 1],
                            NW);  // its the NW edge
                }
                i->setPhysicalNeighbour(physicalGrid_[y][Parameters.getSizeX()
                                                         - 1], W);  // set west direction
            }
            if (y == 0) {
                if (x != 0) {
                    i->setPhysicalNeighbour(physicalGrid_[Parameters.getSizeY()
                                                          - 1][x - 1], SW);   // set south west direction
                }       // edge already setted
                if (x != Parameters.getSizeX() - 1) {
                    i->setPhysicalNeighbour(physicalGrid_[Parameters.getSizeY()
                                                          - 1][x + 1], SE);  // set south east direction
                }       // edge already setted
                i->setPhysicalNeighbour(physicalGrid_[Parameters.getSizeY() -
                                                      1][x], S);  // set south direction
            }
        }
    }
}

void GeometryHandler::connectGeometryNodesToPhysicalNodes() {
    typedef std::map<unsigned int,std::shared_ptr<nodes::GeometryNode> >::const_iterator MapIterator;
    std::vector<MapIterator> helper_vector;

    // create a helper vector for parallelization:
    for (MapIterator it=this->geometry_.getGeometryNodes().begin();
         it!=this->geometry_.getGeometryNodes().end();
         ++it) {
        helper_vector.push_back(it);
    }

#pragma omp parallel for schedule(static)
    for (size_t it=0; it<helper_vector.size(); ++it) {
        this->connectGeometryNodesToPhysicalNodes(helper_vector[it]->second);
    }
}

void GeometryHandler::connectGeometryNodesToPhysicalNodes(
        std::shared_ptr<nodes::GeometryNode> pt) {
    if (!pt->isUpdateNeeded())
        return;

    double minX = pt->getXPos() - 2.0;
    double maxX = pt->getXPos() + 2.0;
    double minY = pt->getYPos() - 2.0;
    double maxY = pt->getYPos() + 2.0;

    if (minX < 0) {
        lbm_fail(
                    "geometry point moved too close to the west boundary");
    }

    if (minY < 0) {
        lbm_fail(
                    "geometry point moved too close to the south boundary");
    }

    if (maxX > Parameters.getSizeX()) {
        lbm_fail(
                    "geometry point moved too close to the east boundary");
    }

    if (maxY > Parameters.getSizeY()) {
        lbm_fail(
                    "geometry point moved too close to the north boundary");
    }


    for (int x = std::ceil(minX); x <= std::floor(maxX); x++) {
        for (int y = std::ceil(minY); y <= std::floor(maxY); y++) {
            pt->addPhysicalNode(this->physicalGrid_[y][x], (x * 4 + y) % 16);
        }
    }
}

void GeometryHandler::generateBoundaryNodes() {
    std::vector<std::shared_ptr<Connection> > helper_vector =
            this->geometry_.getConnections();

    /**
     *@todo fails with pragma, don't know why...
     */
//#pragma omp parallel for schedule(static)
    for (size_t it=0;
         it<helper_vector.size();
         ++it) {
        helper_vector[it]->generateBoundaryNodes(this->physicalGrid_, this->boundaryNodes_);
    }
}
}   // end namespace
}  // end namespace
