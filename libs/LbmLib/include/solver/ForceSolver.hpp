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
#ifndef FORCESOLVER_HPP
#define FORCESOLVER_HPP

#include <LbmLib/include/solver/AbstractForceStruct.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace LbmLib {
namespace nodes {
class GeometryNode;
}

namespace solver {
typedef std::shared_ptr< AbstractForceStruct> shptr_forcestruct;
typedef std::vector< shptr_forcestruct > vec_shptr_forcestruct;
typedef std::map< unsigned int, vec_shptr_forcestruct > map_forcestruct;
/**
 * @brief The actual force solver
 */
class ForceSolver {
 public:
    /**
     * @brief calculates the Force
     * @param nodes A map containing the GeometryNodes on which the forces are applied to.
      */
    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes);

    /**
     * @brief The constructor
     */
    ForceSolver();

    /**
     * @brief loading of the force solver
     * @param filename the file where the forces are stored
     */
    void loadForceFile(const std::string& filename);

    /**
     * @brief add a force
     * @param forcedescriptor as read from the force file
     */
    void addForce(std::stringstream* const forcedescriptor);

    /**
     * @brief writes the forces to the file
     * @param filename the file where the forces are stored
     */
    void writeForceSolver(const std::string& filename) const;

    /**
     * @brief reset all forces
     */
    void deleteAllForces();

    /**
     * @brief reset all forces
     * @param forcetype Delete only force of type.
     */
    void deleteForceType(const unsigned int forcetype);

    /**
     * @brief get all forces of nodeid
     * @param forcevector a vector to store the ForceStructs in
     * @param nodeid the id of the *GeometryNode*
     * @return if forcestructs exist on nodeid, they're returned.
     */
    vec_shptr_forcestruct getForcesOfNode(vec_shptr_forcestruct &forcevector,
                                          const unsigned int nodeid);

    /**
     * @brief getAllForces returns the entire map {nodeid, AbstractForceStruct}
     * @return a map with {nodeid, AbstractForceStruct}
     */
    const map_forcestruct getAllForces() const;


    /**
     * @brief The destructor
     */
    ~ForceSolver();

 private:
    /**
     * @brief storage of the forces
     */
    vec_shptr_forcestruct forces_;

    /**
     * @brief forcemap_ the *AbstractForceStruct*s with the nodeid as key: {nodeid, AbstractForceStruct}
     */
    map_forcestruct forcemap_;
};
}
}  // end namespace

#endif  // FORCESOLVER_HPP
