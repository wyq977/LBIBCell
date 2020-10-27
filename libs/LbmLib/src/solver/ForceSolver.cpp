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
#include <LbmLib/include/Field.hpp>
#include <LbmLib/include/nodes/EulerianPoint.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/solver/ForceSolver.hpp>
#include <LbmLib/include/solver/ForceStructs.hpp>
#include <UtilLib/include/Exception.hpp>
#include <omp.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>

namespace LbmLib {
namespace solver {

ForceSolver::ForceSolver()
{}

void ForceSolver::calculateForce(const std::map<unsigned int,
                std::shared_ptr<nodes::GeometryNode> >& nodes) {
    std::vector< std::shared_ptr<AbstractForceStruct> > tempforcevector;

    const size_t nodes_length = nodes.size();
    typedef std::map<unsigned int, std::shared_ptr<nodes::GeometryNode> >::const_iterator temp_map_it;
    std::vector<temp_map_it> helper_nodes;

    // create the array of iterators sequentially:
    auto nodes_it = nodes.begin();
    for (size_t j=0;
         j<nodes_length;
         ++j)
    {
        helper_nodes.push_back(nodes_it++);
    }

    // reset all forces on the GeometryNodes:
#pragma omp parallel for schedule(static)
    for (size_t it=0; it<nodes_length; ++it) {
        helper_nodes[it]->second->setForce(Field<double>(0.0, 0.0));
    }

    auto forcemap_length = this->forcemap_.size();
    std::vector<map_forcestruct::iterator> helper_forcemap;

    // create the array of iterators sequentially:
    auto forcemap_it = this->forcemap_.begin();
    for (size_t j=0;
         j<forcemap_length;
         ++j) {
        helper_forcemap.push_back(forcemap_it++);
    }

    // compute forces (parallelized):
# pragma omp parallel for schedule(static)
    for (size_t it=0;
         it<forcemap_length;
         ++it) {
        for (auto in : helper_forcemap[it]->second) {
            in->calculateForce(nodes);
        }
    }
}

ForceSolver::~ForceSolver() {}

void ForceSolver::loadForceFile(const std::string& filename) {
    std::ifstream fileStream;
    fileStream.open(filename);
    if (fileStream.is_open()) {
        std::string line;
        while (std::getline(fileStream, line)) {
            std::stringstream lineStream(line);

            if (line.at(0) != '#') {
                this->addForce(&lineStream);
            }
        }
    } else {
        lbm_fail("Cannot find the force solver file.");
    }
    fileStream.close();
}

void ForceSolver::addForce(std::stringstream * const forcedescriptor)
{
    unsigned int type, nodeid;
    std::stringstream copyforcedescriptor(forcedescriptor->str()); // copy
    copyforcedescriptor >> type >> nodeid;
    if (type == 0) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType0(forcedescriptor));
    } else if (type == 1) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType1(forcedescriptor));
    } else if (type == 2) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType2(forcedescriptor));
    } else if (type == 3) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType3(forcedescriptor));
    } else if (type == 4) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType4(forcedescriptor));
    } else if (type == 5) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType5(forcedescriptor));
    } else if (type == 6) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType6(forcedescriptor));
    } else if (type == 7) {
#pragma omp critical
        this->forcemap_[nodeid].push_back(LbmLib::solver::ForceStructs::loadForceType7(forcedescriptor));
    } else {
        lbm_fail("Invalid force solver type found.");
    }
}

void ForceSolver::writeForceSolver(const std::string& filename) const {
    std::ofstream fileStream;
    fileStream.open(filename);
    if (fileStream.is_open()) {
        for (auto it1=this->forcemap_.begin();
                 it1!=this->forcemap_.end();
                 ++it1) {
            for (auto it2 : it1->second) {
                it2->writeForceStruct(&fileStream);
            }
        }
    } else {
        throw UtilLib::Exception(
                "Problem to open the output file for the force solver");
    }
}

void ForceSolver::deleteAllForces()
{
    for (auto it : this->forcemap_) {
        it.second.clear();
        assert(it.second.size()==0 && "ForceSolver::deleteAllForces: couldn't clear forcemap_");
    }
    this->forcemap_.clear();
    assert(this->forcemap_.size()==0 && "ForceSolver::deleteAllForces: couldn't clear forcemap_");
}

void ForceSolver::deleteForceType(const unsigned int forcetype)
{
    // omp can't deal with non-random-access-iterators :(
    const auto forcemap_length = this->forcemap_.size();

    std::vector<map_forcestruct::iterator> helper_map;
    auto map_it = this->forcemap_.begin();

    // create array of iterators sequentally:
    for (size_t j=0;
         j<forcemap_length;
         ++j) {
        helper_map.push_back(map_it++);
    }

/**
  * @todo fails with pragma, of course. other ways?
  */
#pragma omp parallel for schedule(static)
    for (size_t j=0;
         j<forcemap_length;
         ++j) {
        // erase-remove idiom:
        helper_map[j]->second.erase( std::remove_if(helper_map[j]->second.begin(),
                                                    helper_map[j]->second.end(),
                                                    [forcetype](std::shared_ptr<AbstractForceStruct> force){
                                                    if (force->getType() == forcetype) {
                                                      return true;
                                                    }
                                                    else {
                                                      return false;
                                                    }
                                                    }),
                                                    helper_map[j]->second.end()
                                                    );
    }
}

std::vector<std::shared_ptr<AbstractForceStruct> > ForceSolver::getForcesOfNode(std::vector<std::shared_ptr<AbstractForceStruct> > &forcevector,
                                                                                const unsigned int nodeid)
{
    forcevector.clear();
    assert(forcevector.size()==0 && "ForceSolver::getForcesOfNode: non-empty forcevector");

    if ( this->forcemap_.find(nodeid) == this->forcemap_.end() ) { // no forces
        return forcevector;
    } else {
        forcevector.insert(forcevector.end(),
                           this->forcemap_[nodeid].begin(),
                           this->forcemap_[nodeid].end()
                           );
    }
    return forcevector;
}

const map_forcestruct ForceSolver::getAllForces() const
{
    return this->forcemap_;
}

void ForceSolver::deleteForcesAssociatedWithNodeOn(const unsigned int nodeid)
{
    // omp can't deal with non-random-access-iterators :(
    const auto forcemap_length = this->forcemap_.size();

    std::vector<map_forcestruct::iterator> helper_map;
    auto map_it = this->forcemap_.begin();

    // create array of iterators sequentally:
    for (size_t j=0;
         j<forcemap_length;
         ++j) {
        helper_map.push_back(map_it++);
    }

    // first clear all secondary associations, i.e. forcestructs where nodeid is only interaction partner:
    /**
    * @todo fails with pragma, of course. other ways?
    */
#pragma omp parallel for schedule(static)
    for (size_t j=0;
         j<forcemap_length;
         ++j) {
        // erase-remove idiom:
        helper_map[j]->second.erase( std::remove_if(helper_map[j]->second.begin(),
                                                    helper_map[j]->second.end(),
                                                    [nodeid](std::shared_ptr<AbstractForceStruct> force){
                                                    if (force->getPartnerGeometryNode() == nodeid) {
                                                      return true;
                                                    }
                                                    else {
                                                      return false;
                                                    }
                                                    }),
                                                    helper_map[j]->second.end()
                                                    );
    }

    // then clear also all primary associations:
    this->forcemap_[nodeid].clear();
    this->forcemap_.erase(nodeid);
}


}
}  // end namespace

