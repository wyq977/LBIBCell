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
#ifndef ABSTRACTSOLVER_HPP
#define ABSTRACTSOLVER_HPP

#include <LbmLib/include/Direction.hpp>
#include <string>

namespace LbmLib {
namespace solver {
/**
 * @brief the base class of the cde and fluid solvers
 */
class AbstractSolver {
 public:
    /**
     * @brief ~AbstractSolver Destructor
     */
    virtual ~AbstractSolver() {}

    /**
     * @brief initSolver Use this to initalise the solver
     */
    virtual void initSolver() = 0;

    /**
     * @brief collide The collision step of the LBM
     */
    virtual void collide() = 0;

    /**
     * @brief advect The advect step of the LBM
     */
    virtual void advect() = 0;

    /**
     * @brief loads the solver from the stream
     * @param stream the stream where the solver is loaded from
     */
    virtual void loadSolver(std::stringstream* const stream) = 0;

    /**
     * @brief writes the solver to the stream
     * @param stream the stream where the solver is written to
     * @attention the first two parameters must be the x and y position
     */
    virtual void writeSolver(std::ostream* const stream) = 0;

    /**
     * @brief accessDistribution Access to the distribution
     * @param dir the direction where the Distribution is wanted
     * @return A Reference to the Distribution
     */
    virtual double& accessDistribution(const Direction& dir) = 0;

    /**
     * @brief Rescales all distributions by a factor.
     * @param factor The rescaling ractor
     */
    virtual void rescaleDistributions(const double factor) = 0;

    /**
     * @brief setTau Setter method for the tau parameter of the solver
     * @param tau The new value of tau
     */
    void setTau(double tau) {
        tau_ = tau;
    }

    /**
     * @brief getTau Getter method for the tau parameter
     * @return The tau of this solver
     */
    double getTau() const {
        return tau_;
    }

 protected:
    /**
     * @brief AbstractSolver Protected to disable direct instantiation.
     */
    AbstractSolver();

 private:
    /**
     * @brief tau_ The Tau parameter of the solver
     */
    double tau_;
};
}
}  // end namespace

#endif  // ABSTRACTSOLVER_HPP
