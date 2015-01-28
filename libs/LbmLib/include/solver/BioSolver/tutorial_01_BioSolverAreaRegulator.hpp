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
#ifndef TUTORIAL_01_BIOSOLVERAREAREGULATOR_HPP
#define TUTORIAL_01_BIOSOLVERAREAREGULATOR_HPP

#include <LbmLib/include/solver/BioSolver/BioBaseSolver.hpp>
#include <string>
#include <vector>

namespace LbmLib {
namespace solver {

/**
 * @brief The tutorial_01_BioSolverAreaRegulator class. It controls the area and growth of the cells.
 */
class tutorial_01_BioSolverAreaRegulator : public BioBaseSolver<tutorial_01_BioSolverAreaRegulator> {
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
     * @brief tutorial_01_BioSolverAreaRegulator The constructor has to be private to avoid instantiation of this class.
     */
    tutorial_01_BioSolverAreaRegulator();

    /**
     * @brief targetareamap_ Stores the target areas of the cells {domainID,targetarea}.
     */
    std::map<unsigned int,double> targetareamap_;

    /**
     * @brief name The name of this solver.
     */
    static const std::string name;
};
}
}  // end namespace

#endif  // TUTORIAL_01_BIOSOLVERAREAREGULATOR_HPP


