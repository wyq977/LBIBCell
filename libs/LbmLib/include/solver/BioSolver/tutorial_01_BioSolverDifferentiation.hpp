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
#ifndef TUTORIAL_01_BIOSOLVERDIFFERENTIATION_HPP
#define TUTORIAL_01_BIOSOLVERDIFFERENTIATION_HPP

#include <LbmLib/include/solver/BioSolver/BioBaseSolver.hpp>
#include <string>
#include <vector>
#include <memory>

namespace LbmLib {
namespace geometry {
class Connection;
}
namespace solver {

/**
 * @brief The tutorial_01_BioSolverDifferentiation class. It differentiates the celltypes according to user-specified rules.
 */
class tutorial_01_BioSolverDifferentiation : public BioBaseSolver<tutorial_01_BioSolverDifferentiation> {
 public:
    /**
     * @brief Applies biological processes.
     * @param geometryhandler The GeometryHandler.
     * @param forcesolver The ForceSolver.
     */
    virtual void applyBioProcess(
            geometry::GeometryHandler& geometryhandler,
            solver::ForceSolver& forcesolver
            );

 private:
    /**
     * @brief BioBaseSolver has to be a friend class.
     */
    friend class BioBaseSolver;

    /**
     * @brief Make the constructor private to avoid instantiation of this class.
     */
    tutorial_01_BioSolverDifferentiation();

    /**
     * @brief cellDefinition_ The *Connection*s are in a map. First dimension: cell identity. Second dimension: the *Connection*s defining that cell.
     */
    std::map<unsigned int,std::vector<std::shared_ptr<LbmLib::geometry::Connection> > > cellDefinition_;

    /**
     * @brief name The name of this solver.
     */
    static const std::string name;
};
}
}  // end namespace

#endif  // TUTORIAL_01_BIOSOLVERDIFFERENTIATION_HPP


