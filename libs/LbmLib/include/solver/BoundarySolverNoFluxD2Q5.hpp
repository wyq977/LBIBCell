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
#ifndef BOUNDARYSOLVERNOFLUXD2Q5_HPP
#define BOUNDARYSOLVERNOFLUXD2Q5_HPP

#include <LbmLib/include/solver/BoundaryAbstractSolver.hpp>
#include <string>

namespace LbmLib {
namespace solver {
/**
 * @brief a implementaion of a no flux boundary
 */
class BoundarySolverNoFluxD2Q5 : public BaseBoundarySolver<BoundarySolverNoFluxD2Q5> {
 public:
    /**
     * @brief postAdvect Executed after the advection step
     */
    virtual void postAdvect();

    /**
     * @brief preAdvect Executed before the advection step
     */
    virtual void preAdvect();

    /**
     * @brief initSolver Use this to initalise the solver
     */
    virtual void initSolver();

    /**
     * @brief advect The advect step of the LBM
     */
    virtual void advect();

    /**
     * @brief accessDistribution Access to the distribution
     * @param dir the direction where the Distribution is wanted
     * @return A Reference to the Distribution
     */
    virtual double& accessDistribution(const Direction& dir);

 private:
    friend class BaseBoundarySolver;

    BoundarySolverNoFluxD2Q5();
    /**
     * @brief name The name of this solver
     */
    static const std::string name;
    /**
     * @brief distribution_ The Distribution of this boundary condition
     */
    double distribution_;
};
}
}  // end namespace
#endif  // BOUNDARYSOLVERNOFLUXD2Q5_HPP
