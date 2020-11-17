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
#ifndef CDESOLVERD2Q5_HPP
#define CDESOLVERD2Q5_HPP

#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>
#include <array>
#include <string>
#include <iomanip>

namespace LbmLib {
namespace solver {
/**
 * @brief the cde solver for the D2Q5 grid
 */
class CDESolverD2Q5 : public BaseCDESolver<CDESolverD2Q5> {
 public:
    /**
     * @brief ~CDESolverD2Q5 virtual Destructor
     */
    virtual ~CDESolverD2Q5() {}

    /**
     * @brief collide The collision step of the LBM
     */
    virtual void collide();

    /**
     * @brief advect The advect step of the LBM
     */
    virtual void advect();

    /**
     * @brief initSolver Use this to initalise the solver
     */
    virtual void initSolver();

    /**
     * @brief calculateEquilibrium calculates the equilibirum for direction dir
     * @param dir the direction
     * @return the Equilibrium
     */
    virtual double calculateEquilibrium(const Direction& dir);

    /**
     * @brief accessDistribution Access to the distribution
     * @param dir the direction where the Distribution is wanted
     * @return A Reference to the Distribution
     */
    virtual double& accessDistribution(const Direction& dir);

    /**
     * @brief Rescales all distributions by a factor.
     * @param factor The rescaling factor.
     */
    virtual void rescaleDistributions(const double factor);

    /**
     * @brief setDistribution Manually set distribution
     * @param dir the direction where the Distribution is wanted
     * @param newDistribution new direction calculated
     */
    virtual void setDistribution(const Direction& dir, const double newDistribution);

    /**
     * @brief getC Calculates the concentration on this node
     * @return The concentration on this node
     */
    virtual double getC() const;

    /**
     * @brief reinitialise this solver iff the corresponding physical node has switched domain
     */
    virtual void reinitialise();

    /**
     * @brief loads the solver from the file
     * @param stream the stream where the solver is loaded from
     */
    virtual void loadSolver(std::stringstream* const stream);

    /**
     * @brief writes the solver to the file
     * @param stream the stream where the solver is written to
     */
    virtual void writeSolver(std::ostream* const stream);

 private:
    /**
     * declare this as a friend call as it should be the only accessing the Constructor
     */
    friend class BaseCDESolver;
    /**
     * @brief CDESolverD2Q5 private Constructor only use create method for instantiation
     */
    CDESolverD2Q5();
    /**
     * @brief localSwap Executes the local swap inpreparation of the advect step
     */
    void localSwap();

    /**
     * @brief distributions_ The distributions of this solver
     */
    std::array<double, 5> distributions_;

    /**
     * @brief cdeDirIter_ Iterator for the CDE Directions
     */
    static const CDEDirectionsIteratorD2Q5 cdeDirIter_;
    /**
     * @brief name_ The name of this solver
     */
    static const std::string name;
};
}
}  // end namespace

#endif  // CDESOLVERD2Q5_HPP
