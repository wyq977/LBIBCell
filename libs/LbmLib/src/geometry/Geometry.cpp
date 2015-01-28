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
#include <LbmLib/include/geometry/Geometry.hpp>

#include <LbmLib/include/Constants.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Log.hpp>

#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cassert>
#include <omp.h>
#include <sys/stat.h>

#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkStringArray.h>
#include <vtkPointData.h>
#include <vtkCellData.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkXMLMultiBlockDataWriter.h>
#include <vtkXMLMultiBlockDataReader.h>

namespace LbmLib {
namespace geometry {

Geometry::Geometry(const std::string& filename) : fastneighborlist_(Parameters.getSizeX(),
                                                                    Parameters.getSizeY(),
                                                                    (unsigned int)std::ceil(Parameters.getSizeX()/(double)MAXBINSIZE),
                                                                    (unsigned int)std::ceil(Parameters.getSizeY()/(double)MAXBINSIZE)), isValidRangeQueryDataStructure_(0) {

    int len = strlen(filename.c_str());
    const char *last_four = &filename.c_str()[len-4];

    if ( strcmp(last_four, ".vtm") == 0 )
    {
        this->loadGeometryVTK(filename);
    }
    else if ( strcmp(last_four, ".txt") == 0)
    {
        this->loadGeometryTXT(filename);
    }
}

Geometry::~Geometry() {
}

void Geometry::writeGeometry(const std::string& fileName) const {
    std::ofstream fileStream;
    fileStream.open(fileName);
    if (fileStream.is_open()) {
        fileStream << "#Nodes (id\txPos\tyPos)\n";
        for (const auto& node : getGeometryNodes()) {
            node.second->writeNode(&fileStream);
        }
        fileStream<<
        "#Connection (nodeId1\tnodeId2\tdomainId\tbsolver\tcdesolver\t...)\n";
        for (const auto& c : getConnections()) {
            c->writeConnection(&fileStream);
        }
    } else {
        lbm_fail("Cannot open the output file for the force solver.");
    }

    fileStream.close();
}
/**
 * @todo remove the registering of the connection in the geometrynodes once the connection registers itself
 */
void Geometry::loadGeometryTXT(const std::string& fileName) {
    std::ifstream fileStream;
    fileStream.open(fileName);
    std::stringstream message;

    this->cellTypeTrackerMap_[0] = 0; // fluid default
    message << "Loading geometry from txt file: cell type not supported; all cell types are set to 1.";
    LOG(UtilLib::logINFO) << message.str().c_str();

    if (this->connections_.size()!=0 ||
            this->geometryNodes_.size()!=0) {
        lbm_fail("Only load the geometry once.");
    }

    if (fileStream.is_open()) {
        std::string line;
        bool nodes = true;
        while (std::getline(fileStream, line)) {
            std::stringstream lineStream(line);
            if (line.find("#Connection") != std::string::npos) {
                nodes = false;
            }
            if (line.at(0) != '#') {
                if (nodes) {
                    // we have nodes
                    unsigned int id;
                    double x, y;
                    lineStream >> id >> x >> y;
                    this->geometryNodes_[id] = std::make_shared<nodes::GeometryNode>(
                            x,
                            y,
                            id);
                    this->fastneighborlist_.addElement( // add the element also to the fastneighborlist
                                this->geometryNodes_[id]
                                );
                } else {
                    // we have connections
                    unsigned int id1, id2, domainId;
                    std::map<std::string,
                            std::vector<std::string> > connectionType;
                    lineStream >> id1 >> id2 >> domainId;
                    this->cellTypeTrackerMap_[domainId] = 1; // JUST SET EVERYTHING TO 1
                    std::string temp1, temp2;
                    while (lineStream >> temp1 >> temp2) {
                        connectionType[temp1].push_back(temp2);
                    }
                    this->connections_.push_back(std::make_shared<Connection>(
                                    geometryNodes_[id1], geometryNodes_[id2],
                                    connectionType, domainId));

                    // also register the connection in the nodes:
                    this->geometryNodes_[id1]->setConnection<1>(this->connections_.back());
                    this->geometryNodes_[id2]->setConnection<0>(this->connections_.back());
                }
            }
        }
    } else {
        lbm_fail("Cannot find the geometry.txt file.");
    }
    fileStream.close();
    this->isValidRangeQueryDataStructure_ = 1; // tag range query data structure as valid
}

void Geometry::loadGeometryVTK(const std::string &fileName)
{
    vtkSmartPointer<vtkMultiBlockDataSet> multiBDS_read =
            vtkSmartPointer<vtkMultiBlockDataSet>::New ();
    vtkSmartPointer<vtkXMLMultiBlockDataReader> reader =
            vtkSmartPointer<vtkXMLMultiBlockDataReader>::New();
    vtkSmartPointer<vtkPolyData> polygonPolyData_extract =
            vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points_extract =
            vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polygons_extract =
            vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkDoubleArray> domainidentifierAttributeArray =
            vtkSmartPointer<vtkDoubleArray>::New();
    vtkSmartPointer<vtkDoubleArray> celltypeAttributeArray =
            vtkSmartPointer<vtkDoubleArray>::New();
    vtkSmartPointer<vtkDataArray> tempArray;
    vtkSmartPointer<vtkAbstractArray> tempAbstractArray;
    vtkSmartPointer<vtkStringArray> stringAttribute_extract =
            vtkSmartPointer<vtkStringArray>::New();

    unsigned int blockId_polygon;
    unsigned int blockId_count = 0;
    double* COORD; // an array to store coordinates

    vtkIdType numPolygons;
    vtkIdType cellLocation = 0; // the index in the cell array
    vtkIdType numIds; // to hold the size of the cell
    vtkIdType *pointIds; // to hold the ids in the cell

    std::map<std::string,
            std::vector<std::string> > connectionType;
    unsigned int id1,id2,domainId,cellType; // for the connections
    std::string temp1,temp2;
    vtkStdString boundarystring;

    this->cellTypeTrackerMap_[0] = 0; // fluid default

    reader->SetFileName(fileName.c_str());
    struct stat buffer;
    if (stat (fileName.c_str(), &buffer) == 0) {
        reader->Update();
        multiBDS_read->ShallowCopy(reader->GetOutput());

        for (unsigned int k=0; k<multiBDS_read->GetNumberOfBlocks(); ++k) { // loop all blocks to find the polygon block
            polygonPolyData_extract->ShallowCopy(multiBDS_read->GetBlock(k)); // extract the block k
            if ( strcmp(multiBDS_read->GetBlock(k)->GetClassName(),"vtkPolyData")==0 ) { // candidate blocks have to be vtkPolyData
                polygons_extract = polygonPolyData_extract->GetPolys(); // extract the polygons
                if (polygons_extract->GetNumberOfCells() > 0) { // this is only the right block if there are some polygons in it
                    blockId_polygon = k; // save the blockId
                    blockId_count++;
                }
            }
        }

        if (blockId_count != 1) {
            lbm_fail("more or less than one vtk polygon block found in input geometry file found.");
        }

        polygonPolyData_extract->ShallowCopy(multiBDS_read->GetBlock(blockId_polygon)); // extract the block with the polygons
        points_extract = polygonPolyData_extract->GetPoints(); // extract the points
        polygons_extract = polygonPolyData_extract->GetPolys();
        numPolygons = polygons_extract->GetNumberOfCells();

        // add the points:
        for (int k=0; k<points_extract->GetNumberOfPoints(); ++k) {
            COORD = points_extract->GetPoint(k);

            this->geometryNodes_[k] = std::make_shared<nodes::GeometryNode>(
                        COORD[0],
                        COORD[1],
                        k);
            this->fastneighborlist_.addElement( // add the element also to the fastneighborlist
                        this->geometryNodes_[k]
                        );
        }

        // extract the domain_identifier identifier flags (which are vtkPoint attributes):
        tempArray = polygonPolyData_extract->GetPointData()->GetArray("domain_identifier");
        if (tempArray==NULL) {
            lbm_fail("domain_identifier array not found in vtk input file.");
        }
        domainidentifierAttributeArray->DeepCopy(tempArray);

        // extract the cell_type identifier flags (which are vtkPoint attributes):
        tempArray = polygonPolyData_extract->GetPointData()->GetArray("cell_type");
        if (tempArray==NULL) {
            lbm_fail("cell_type array not found in vtk input file.");
        }
        celltypeAttributeArray->DeepCopy(tempArray);

        // extract the boundary_descriptor attributes:
        tempAbstractArray = polygonPolyData_extract->GetPointData()->GetAbstractArray("boundarydescriptors");
        if (tempAbstractArray==NULL) {
            lbm_fail("boundarydescriptor array not found in vtk input file");
        }
        stringAttribute_extract->DeepCopy(tempAbstractArray);

        // add the connections:
        for (vtkIdType i = 0; i < numPolygons; i++) // loop all polygons
        {
            polygons_extract->GetCell(cellLocation, numIds, pointIds) ;
            cellLocation += 1 + numIds;

            assert((unsigned int)domainidentifierAttributeArray->GetSize() == (unsigned int)this->geometryNodes_.size());
            domainId = domainidentifierAttributeArray->GetValue(pointIds[0]); // we just take the first value and do not check for consistency

            assert((unsigned int)domainidentifierAttributeArray->GetSize() == (unsigned int)this->geometryNodes_.size());
            cellType = celltypeAttributeArray->GetValue(pointIds[0]); // we just take the first value and do not check for consistency
            this->cellTypeTrackerMap_[domainId] = cellType; // feed the map

            for (int k=0; k<numIds-1; ++k) { // loop the points of polygon i
                id1 = pointIds[k];
                id2 = pointIds[k+1];

                boundarystring = stringAttribute_extract->GetValue(pointIds[k]); // get the boundary string
                std::stringstream stringattributestringstream(boundarystring); // create a stringstream for parsing

                connectionType.clear();
                while (stringattributestringstream >> temp1 >> temp2) { // parse pairs of strings
                    connectionType[temp1].push_back(temp2);
                }

                this->connections_.push_back(std::make_shared<Connection>(
                                geometryNodes_[id1], geometryNodes_[id2],
                                connectionType, domainId));

                // also register the connection in the nodes:
                this->geometryNodes_[id1]->setConnection<1>(this->connections_.back());
                this->geometryNodes_[id2]->setConnection<0>(this->connections_.back());
            }

            this->connections_.push_back(std::make_shared<Connection>(
                            geometryNodes_[pointIds[numIds-1]], geometryNodes_[pointIds[0]],
                            connectionType, domainId)); // close the polygon

            // close the polygon: also register the connection in the nodes:
            this->geometryNodes_[pointIds[numIds-1]]->setConnection<1>(this->connections_.back());
            this->geometryNodes_[pointIds[0]]->setConnection<0>(this->connections_.back());
        }
    }
    else {
        lbm_fail("Cannot find the geometry.vtm file.");
    }
}

const std::vector<std::shared_ptr<nodes::GeometryNode> >
Geometry::getGeometryNodesWithinRadius(const double x,
                                       const double y,
                                       const double radius) const {
    if (this->isValidRangeQueryDataStructure_ != 1) {
        this->reconstructRangeQueryDataStructure();
    }
    std::vector<std::shared_ptr<nodes::GeometryNode> > queryresult;

    this->fastneighborlist_.getGeometryNodesWithinRadius(
                queryresult,
                x,
                y,
                radius);
    return queryresult;
}

const std::vector<std::shared_ptr<nodes::GeometryNode> >
Geometry::getGeometryNodesWithinRadiusWithAvoidance(const double x,
                                                    const double y,
                                                    const double radius,
                                                    const unsigned int avoidDomainID) const
{
    if (this->isValidRangeQueryDataStructure_ != 1) {
        this->reconstructRangeQueryDataStructure();
    }
    std::vector<std::shared_ptr<nodes::GeometryNode> > queryresult;

    this->fastneighborlist_.getGeometryNodesWithinRadiusWithAvoidance(
                queryresult,
                x,
                y,
                radius,
                avoidDomainID);


    std::vector<std::shared_ptr<nodes::GeometryNode> > temp =
            this->getGeometryNodesWithinRadius(x,y,radius);

    return queryresult;
}

std::shared_ptr<nodes::GeometryNode>
Geometry::getGeometryNodesWithinRadiusWithAvoidanceClosest(const double x,
                                                 const double y,
                                                 const double radius,
                                                 const unsigned int avoidDomainID) const
{
    if (this->isValidRangeQueryDataStructure_ != 1) {
#pragma omp critical
        this->reconstructRangeQueryDataStructure();
    }
    std::vector<std::shared_ptr<nodes::GeometryNode> > queryresult;
    this->fastneighborlist_.getGeometryNodesWithinRadiusWithAvoidanceClosest(
                queryresult,
                x,
                y,
                radius,
                avoidDomainID);

    if (queryresult.size() == 1) {
        return queryresult[0];
    }
    else {
        return nullptr;
    }
}

const std::vector<std::shared_ptr<Connection> >& Geometry::getConnections()
const {
    return connections_;
}

unsigned int Geometry::addGeometryNode(const double x,const double y) {
    const unsigned int id = this->geometryNodes_.rbegin()->first+1; // bump new id
    this->geometryNodes_[id] =
            std::make_shared<nodes::GeometryNode>(
            x,
            y,
            id);
    this->fastneighborlist_.addElement( // also add the ptr to the fastneighborlist
                this->geometryNodes_[id]
                );
    if (this->geometryNodes_.rbegin()->first != id) {
        lbm_fail("Could not create and add the GeometryNode.");
    }
    std::stringstream message;
    message << "GeometryNode with nodeid=" << id
            <<   " added at ("<< x << "," << y << ")";
    LOG(UtilLib::logINFO) << message.str().c_str();
    return id;
}

unsigned int Geometry::removeGeometryNode(const unsigned int nodeid)
{
    try {
        std::shared_ptr<Connection> C1 = this->geometryNodes_[nodeid]->getConnection<0>(); // incoming connection
        std::shared_ptr<Connection> C2 = this->geometryNodes_[nodeid]->getConnection<1>(); // outgoing connection
        std::shared_ptr<nodes::GeometryNode> N1 = C1->getGeometryNodes().first;
        std::shared_ptr<nodes::GeometryNode> N2 = C2->getGeometryNodes().second;
        std::map<std::string, std::vector<std::string> > descriptor = C1->getBoundaryConditionDescriptor();
        unsigned int domainid = this->geometryNodes_[nodeid]->getDomainIdOfAdjacentConnections();

        // remove old *Connection*s:
        this->eraseConnection(C1);
        this->eraseConnection(C2);

        // remove old node:
        this->fastneighborlist_.removeElement( // remove the element from the fastneighborlist
                    this->geometryNodes_[nodeid]
                    );
        this->geometryNodes_.erase(nodeid); // remove from the primary data structure

        // add the new connection:
        this->addConnection(N1,N2,descriptor,domainid); ///< @todo some error here; domainid is automatically bumped
    }
    catch (...){
        lbm_fail("Geometry::removeGeometryNode: Could not remove the GeometryNode");
    }
    return 1;
}

/**
 * @todo remove the registering of the connection in the geometrynodes once the connection registers itself
 */
void Geometry::addConnection(std::shared_ptr<nodes::GeometryNode> p1,
                             std::shared_ptr<nodes::GeometryNode> p2,
                             const std::map<std::string, std::vector<std::string> > boundaryConditionDescriptor,
                             const unsigned int domainIdentifier) {
#pragma omp critical
    {
    this->connections_.push_back(std::make_shared<Connection>(
                                     p1,
                                     p2,
                                     boundaryConditionDescriptor,
                                     domainIdentifier));
    // also register the connection in the nodes:
    p1->setConnection<1>(this->connections_.back());
    p2->setConnection<0>(this->connections_.back());
    }

    assert(p1->getConnection<1>() != nullptr);
    assert(p2->getConnection<0>() != nullptr);
    LOG(UtilLib::logINFO) << "Connection added";
}

void Geometry::moveGeometryNodes()
{
    std::vector<std::shared_ptr<nodes::GeometryNode> > tempvector;

    // bring into vectorform for parallelization:
    for (auto it : this->geometryNodes_) {
        tempvector.push_back(it.second);
    }

    // move:
#pragma omp parallel for schedule(dynamic)
    for (size_t it=0; it<tempvector.size(); ++it) {
        tempvector[it]->move();
    }
    this->isValidRangeQueryDataStructure_ = 0;
}

void Geometry::eraseConnection(std::shared_ptr<Connection> toDelete) {
    /**
     * @todo this has to be tested!!!!!!
     */
#pragma omp critical
    {
    if (toDelete->getGeometryNodes().first->getConnection<1>()
            == toDelete) { // only delete if it's still me (a new connection might already be subscribed)
        toDelete->getGeometryNodes().first->setConnection<1>(nullptr);
    }
    if (toDelete->getGeometryNodes().second->getConnection<0>()
            == toDelete) { // only delete if it's still me (a new connection might already be subscribed)
        toDelete->getGeometryNodes().second->setConnection<0>(nullptr);
    }
    this->connections_.erase(std::remove(this->connections_.begin(),
                                         this->connections_.end(),
                                         toDelete),
                             this->connections_.end());
    LOG(UtilLib::logDEBUG1) << "Connection erased.";
    }
}

bool Geometry::checkGeometryIntegrity() const
{
    try {
        for (auto it : this->geometryNodes_) {
            if (it.second != it.second->getConnection<0>()->getGeometryNodes().second) {
                lbm_fail("Geometry::checkGeometryIntegrity");
            }

            if (it.second != it.second->getConnection<1>()->getGeometryNodes().first) {
                lbm_fail("Geometry::checkGeometryIntegrity");
            }
        }

        for (auto it : this->connections_) {
            if (it != it->getGeometryNodes().first->getConnection<1>()) {
                lbm_fail("Geometry::checkGeometryIntegrity");
            }

            if (it != it->getGeometryNodes().second->getConnection<0>()) {
                lbm_fail("Geometry::checkGeometryIntegrity");
            }
        }

        if (this->connections_.size() != this->geometryNodes_.size()) {
            lbm_fail("Geometry::checkGeometryIntegrity: #connections != #geometrynodes");
        }
    }
    catch (...) {
        lbm_fail("Geometry::checkGeometryIntegrity: Couln't check geometry integrity.");
    }
    return 1;
}

void Geometry::invalidateRangeQueryDataStructure()
{
    this->isValidRangeQueryDataStructure_ = 0;
}

void Geometry::reconstructRangeQueryDataStructure() const
{
    this->fastneighborlist_.reset(this->geometryNodes_);
    this->isValidRangeQueryDataStructure_ = 1;
}

std::map<unsigned int, unsigned int> Geometry::getCellTypeTrackerMap() const // pass by value
{
    return this->cellTypeTrackerMap_;
}


}   // end namespace
}  // end namespace
