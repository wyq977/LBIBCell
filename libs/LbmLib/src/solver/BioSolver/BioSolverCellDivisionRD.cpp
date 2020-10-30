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
#include <LbmLib/include/solver/BioSolver/BioSolverCellDivisionRD.hpp>
#include <LbmLib/include/geometry/Connection.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <random>

namespace LbmLib {
namespace solver {
namespace {
    const unsigned int FREQUENCY = 100; ///< The frequency of applying this solver
    const double MAXCELLSIZE = 400.0; ///< The maximal area of cells before division  mean area of Heller 5.34 * 62.8 *2
    const double SD = 50.0; // standard deviation of the normal distribution for MAXCELLSIZE
    const double TRUNC = 0.5; // Degree of truncation, i.e. 0.5 is 50% cells will not be available 


}
namespace bg = boost::geometry;

BioSolverCellDivisionRD::BioSolverCellDivisionRD() : BioBaseSolver()
{}

void BioSolverCellDivisionRD::applyBioProcess(geometry::GeometryHandler& geometryhandler,
                                            ForceSolver &forcesolver
                                            ) {
    std::map<unsigned int,double> areas;

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // step 1a: update cell definition trackings
    this->updateCellDefinition(geometryhandler);

    // step 1b: initialize cellTypeTrackereMap iff not yet done
    if (geometryhandler.getCellTypeTrackerMap().size() == 0) {
        geometryhandler.getCellTypeTrackerMap().clear();
        geometryhandler.getCellTypeTrackerMap()[0] = 0;  // fluid default
        for (auto it : this->cellDefinition_) { // default all other cells to type 1
            geometryhandler.getCellTypeTrackerMap()[it.first] = 1;
        }
    }

    // step 2: compute areas
    areas = geometryhandler.computeAreas();
    areas[0] = 0;

    // step 2a: get randomly distributed AREA_RAND
    // cells can divide only if area > AREA_RAND
    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)
    std::normal_distribution<double> normal(MAXCELLSIZE,SD); // normal distribution
    auto AREA_RAND = normal(rng);

    // generate a randomized, truncated vector of pointers to the areas map
    using pair_type = std::pair<const unsigned int, double>;
    std::vector<pair_type *> pair_ptrs;
    for (auto& p : areas)
    {
        pair_ptrs.emplace_back(&p);
    }

    // Reorders the id-area pair vector
    std::random_shuffle(std::begin(pair_ptrs), std::end(pair_ptrs));

    // truncate percentage of cells dividing
    pair_ptrs.resize(std::round(pair_ptrs.size() * TRUNC));

    // step 3: if required, divide cell
    for(auto& it : pair_ptrs){
            if (it->second > AREA_RAND) {
              // step 3a: divide the cell
              this->divideCell(geometryhandler,it->first);

              // step 3b: update cell definition trackings
              this->updateCellDefinition(geometryhandler);

              // step 3c: add the new cell to the cellTypeTrackerMap
              geometryhandler.getCellTypeTrackerMap()[this->cellDefinition_.rbegin()->first] =
                      geometryhandler.getCellTypeTrackerMap()[it->first];;

              // step 3d: cure the Lattice
              geometryhandler.cureLattice();
          }
    }

    //step 4: copy the cell types to the PhysicalGrid
    geometryhandler.copyCellTypeToPhysicalNodes(
                geometryhandler.getCellTypeTrackerMap()
                );
}

void BioSolverCellDivisionRD::divideCell(geometry::GeometryHandler& geometryhandler,unsigned int domainidentifier)
{
    const unsigned int newCellID = this->cellDefinition_.rbegin()->first + 1;
    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > affectedConnections;

    // step 1: find the two connections used for dividing the cell (choose the policy!)
    affectedConnections = this->getTwoConnectionsLongestAxis(domainidentifier);
    //affectedConnections = this->getTwoConnectionsRandomDirection(domainidentifier);

    // step : seal the cut edges:
    affectedConnections[0]->getGeometryNodes().first->setConnection<1>(nullptr);
    affectedConnections[0]->getGeometryNodes().second->setConnection<0>(nullptr);
    affectedConnections[1]->getGeometryNodes().first->setConnection<1>(nullptr);
    affectedConnections[1]->getGeometryNodes().second->setConnection<0>(nullptr);

    // step 2: create the new *Connection*s
    const std::map<std::string, std::vector<std::string> > descriptor1 =
            affectedConnections[0]->getBoundaryConditionDescriptor();
    const std::map<std::string, std::vector<std::string> > descriptor2 =
            affectedConnections[1]->getBoundaryConditionDescriptor();
    if (descriptor1 != descriptor2) {
        lbm_fail("The method cannot handle cell division cases with heterogeneous boundary conditions.");
    }

    geometryhandler.createConnection(
                (*affectedConnections[0]).getGeometryNodes().first,
                (*affectedConnections[1]).getGeometryNodes().second,
                descriptor1,
                domainidentifier);

    geometryhandler.createConnection(
                (*affectedConnections[1]).getGeometryNodes().first,
                (*affectedConnections[0]).getGeometryNodes().second,
                descriptor1,
                domainidentifier);

    // step 3: erase old *Connection*s
    for (auto it : affectedConnections) {
        geometryhandler.eraseConnection(it);
    }

    //step 4: bump domainID's of the connections
    std::shared_ptr<LbmLib::geometry::Connection>  CSTART = (*affectedConnections[1]).getGeometryNodes().second->getConnection<1>();
    std::shared_ptr<LbmLib::geometry::Connection>  C1 = CSTART;
    std::shared_ptr<LbmLib::geometry::Connection>  C2 = nullptr;
    CSTART->setDomainIdentifier(newCellID);
    while (C2 != CSTART) {
        C2 = (*C1).getGeometryNodes().second->getConnection<1>();
        C2->setDomainIdentifier(newCellID);
        C1 = C2;
    }

    LOG(UtilLib::logINFO) << "Cell with domainIdentifier=" <<
                             domainidentifier << " divided" <<
                             " at time " << Parameters.getCurrentIteration();
}

const std::vector<std::shared_ptr<LbmLib::geometry::Connection> >
BioSolverCellDivisionRD::getTwoConnectionsLongestAxis(unsigned int domainidentifier)
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > affectedconnections;
    double dist = 0.0;
    double tempdist = 0.0;
    double invslope = 0.0;
    double intercept = 0.0;
    std::shared_ptr<nodes::GeometryNode> N1 = nullptr;
    std::shared_ptr<nodes::GeometryNode> N2 = nullptr;
    point MIDPOINT;
    bool C1 = 0;
    bool C2 = 0;

    // step 1: find furthest point pair (brute force, O(N²) complexity)
    for (auto it1 : this->cellDefinition_[domainidentifier]) {
        for (auto it2 : this->cellDefinition_[domainidentifier]) {
            tempdist = boost::geometry::distance(*((*it1).getGeometryNodes().first),
                                                 *((*it2).getGeometryNodes().first)
                                                 );
            if (tempdist > dist) {
                dist = tempdist;
                N1 = (*it1).getGeometryNodes().first;
                N2 = (*it2).getGeometryNodes().first;
            }
        }
    }

    // step 2: compute perpendicular axis
    MIDPOINT.set<0>(0.5*(N1->getXPos()+N2->getXPos()));
    MIDPOINT.set<1>(0.5*(N1->getYPos()+N2->getYPos()));
    invslope = -(N2->getXPos()-N1->getXPos()) / (N2->getYPos()-N1->getYPos());
    intercept = MIDPOINT.get<1>() - invslope*MIDPOINT.get<0>();
    assert(std::isfinite(invslope));
    assert(std::isfinite(intercept));

    // step 3: find affected *Connection*s
    for (auto it : this->cellDefinition_[domainidentifier]) {
        C1 = (*it).getGeometryNodes().first->getYPos() > invslope*(*it).getGeometryNodes().first->getXPos() + intercept;
        C2 = (*it).getGeometryNodes().second->getYPos() < invslope*(*it).getGeometryNodes().second->getXPos() + intercept;
        if ( (C1 && C2) || (!C1 && !C2)) {
            affectedconnections.push_back(it);
        }
    }
    assert(affectedconnections.size()==2 && "More or less than 2 connections found in BioSolver::getTwoConnectionsLongestAxis().");
    return affectedconnections;
}

const std::vector<std::shared_ptr<LbmLib::geometry::Connection> >
BioSolverCellDivisionRD::getTwoConnectionsRandomDirection(unsigned int domainidentifier)
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > affectedconnections;
    double invslope = 0.0;
    double intercept = 0.0;
    const int seed = 1; // for debugging
    static std::mt19937 gen(seed); // for debugging
    //static std::random_device rd;
    //static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0,2.0*M_PI); // [rad]
    bool C1 = 0;
    bool C2 = 0;
    Field<double> cm(0,0); ///<center of mass temp

    // step 1: find the center of mass
    for (auto it : this->cellDefinition_[domainidentifier]) {
        cm += (*it).getGeometryNodes().first->getPos();
    }
    cm /= this->cellDefinition_[domainidentifier].size();

    unsigned int counter = 0;
    while (affectedconnections.size()!=2) {
        // step 2: get a random direction and compute intercept
        invslope = std::tan(dis(gen));
        intercept = cm.y - invslope*cm.x;
        assert(std::isfinite(invslope));
        assert(std::isfinite(intercept));
        affectedconnections.clear();

        // step 3: find affected *Connection*s
        for (auto it : this->cellDefinition_[domainidentifier]) {
            C1 = (*it).getGeometryNodes().first->getYPos() > invslope*(*it).getGeometryNodes().first->getXPos() + intercept;
            C2 = (*it).getGeometryNodes().second->getYPos() < invslope*(*it).getGeometryNodes().second->getXPos() + intercept;
            if ( (C1 && C2) || (!C1 && !C2)) {
                affectedconnections.push_back(it);
            }
        }

        counter++;
        if (counter > 10) {
            lbm_fail("could not find cell division plane");
        }
    }

    assert(affectedconnections.size()==2 && "More or less than 2 connections found in BioSolver::getTwoConnectionsLongestAxis().");
    return affectedconnections;
}

void BioSolverCellDivisionRD::updateCellDefinition(geometry::GeometryHandler& geometryhandler)
{
    this->cellDefinition_.clear();
    for (auto it : geometryhandler.getGeometry().getConnections()) {
        this->cellDefinition_[(*it).getDomainIdentifier()].push_back(it);
    }
}

const std::string BioSolverCellDivisionRD::name = "BioSolverCellDivisionRD";
}
}  // end namespace