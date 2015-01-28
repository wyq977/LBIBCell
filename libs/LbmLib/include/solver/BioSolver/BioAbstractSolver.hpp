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
#ifndef BIOABSTRACTSOLVER_HPP
#define BIOABSTRACTSOLVER_HPP
#include <LbmLib/include/solver/BioSolver/BioSolverFactory.hpp>
#include <array>
#include <string>
#include <vector>

namespace LbmLib {

namespace geometry {
class GeometryHandler;
}
namespace solver {
class ForceSolver;
class MassAbstractSolver;
}

namespace solver {
/**
 * @brief The MassAbstractSolver class The abstract base class for all Mass Solvers
 * @attention Do not inherit from this class. Use class \c BaseMassSolver instead, as this will then register the class automatically.
 */
class BioAbstractSolver {
 public:
    /**
     * @brief The virtual Destructor.
     */
    virtual ~BioAbstractSolver() {}

    /**
     * @brief The pure virtual method which applies the biological processes.
     * @param geometryhandler The fluid GeometryHandler.
     * @param forcesolver The forceSolver.
     */
    virtual void applyBioProcess(
            geometry::GeometryHandler& geometryhandler,
            solver::ForceSolver& forcesolver
            ) =
        0;

    /**
     * @brief Get the name of the solver
     * @return the name of the solver
     */
    virtual std::string getName() = 0;

 protected:
    /**
     * @brief Protected Constructor. Only use create() method for instantiation.
     */
    BioAbstractSolver() {}
};

}
}  // end namespace

#endif  // BIOABSTRACTSOLVER_HPP
