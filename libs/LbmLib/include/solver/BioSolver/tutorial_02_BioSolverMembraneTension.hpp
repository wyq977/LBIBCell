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
#ifndef TUTORIAL_02_BIOSOLVERMEMBRANETENSION_HPP
#define TUTORIAL_02_BIOSOLVERMEMBRANETENSION_HPP

#include <LbmLib/include/solver/BioSolver/BioBaseSolver.hpp>
#include <string>
#include <vector>

namespace LbmLib {
namespace solver {

/**
 * @brief The tutorial_02_BioSolverMembraneTension class. It updates the membrane tension forces.
 */
class tutorial_02_BioSolverMembraneTension : public BioBaseSolver<tutorial_02_BioSolverMembraneTension> {
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
     * @brief The BioBaseSolver has to be friend class.
     */
    friend class BioBaseSolver;

    /**
     * @brief tutorial_02_BioSolverMembraneTension The constructor has to be private to avoid instantiation of this class.
     */
    tutorial_02_BioSolverMembraneTension();

    /**
     * @brief name The name of this solver.
     */
    static const std::string name;
};
}
}  // end namespace

#endif  // TUTORIAL_02_BIOSOLVERMEMBRANETENSION_HPP


