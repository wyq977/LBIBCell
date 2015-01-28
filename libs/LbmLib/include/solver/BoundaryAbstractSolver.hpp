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
#ifndef BOUNDARYABSTRACTSOLVER_HPP
#define BOUNDARYABSTRACTSOLVER_HPP
#include <LbmLib/include/Direction.hpp>
#include <LbmLib/include/solver/BoundarySolverFactory.hpp>
#include <array>
#include <vector>
#include <string>
#include <iostream>

namespace LbmLib {
namespace nodes {
class BoundaryNode;
}
namespace solver {
/**
 * @brief The BoundaryAbstractSolver class The abstract base class for all Boundary Solvers
 * @attention Do not inherit from this class use class \c BaseBoundarySolver instead, as this will then register the class automatically
 */
class BoundaryAbstractSolver {
 public:
    /**
     * @brief ~BoundaryAbstractSolver virtual Destructor
     */
    virtual ~BoundaryAbstractSolver() {}

    /**
     * @brief connectPhysicalNode This connects the Solver with an physical node this should be
     * executed before using this class
     * @param boundaryNode The node this solver is connected to.
     */
    void connectBoundaryNode(const nodes::BoundaryNode* boundaryNode);

    /**
     * @brief advect The advect step of the LBM
     */
    virtual void advect() = 0;

    /**
     * @brief postAdvect Executed after the advection step
     */
    virtual void postAdvect() = 0;

    /**
     * @brief preAdvect Executed before the advection step
     */
    virtual void preAdvect() = 0;

    /**
     * @brief initSolver Use this to initalise the solver
     */
    virtual void initSolver() = 0;

    /**
     * @brief connectToCDESolvers makes sure that this boundary Solver is executed on all connected CDE Solvers
     * @param cdeSolvers a vector of strings with the names of the corresponding Solvers
     */
    void connectToCDESolvers(const std::vector<std::string>& cdeSolvers);

    /**
     * @brief accessDistribution Access to the distribution
     * @param dir the direction where the Distribution is wanted
     * @return A Reference to the Distribution
     */
    virtual double& accessDistribution(const Direction& dir) = 0;

 protected:
    /**
     * @brief CDESolver protected Constructor only use create method for instantiation
     */
    BoundaryAbstractSolver();
    /**
     * @brief physicalNode_ The physical Node which owns this solver
     */
    const nodes::BoundaryNode* boundaryNode_;
    /**
     * @brief cdeSolvers_ The cde solver this boundary solver is responsible for.
     */
    std::vector<std::string> cdeSolvers_;
};

/**
 * @brief The Base class for all BoundarySolvers implementations
 * This classes uses the recursive template idiom to automatically register child classes in the factory.
 * To implement a solver inherit from this class and provide the same class as a template argument.
 * Additional the class should provide a static member with the name name which stores a
 * unique name for the class.
 * Preferentially declare the constructor and the static member name private and make the
 * BaseCDESolver a friend class of this.
 * look at /c BaseCDESolver for an example
 */
template <class T>
struct BaseBoundarySolver : BoundaryAbstractSolver {
 protected:
    /**
     * @brief BaseBoundarySolver The Constructor which enforces that the specialisation is done
     */
    BaseBoundarySolver() {
        reg = reg;  // force specialization
    }

    /**
     * @brief ~BaseBoundarySolver virtual Destructor
     */
    virtual ~BaseBoundarySolver() {}

    /**
     * @brief create Static constructor.
     * A pointer to this function along with the
     * name of the algorithm is registered with the Solver Factory.
     * @return Pointer to a new \c AbstractCDESolver instance. Responsibility for
     *         allocated memory lies with the caller.
     */
    static BoundaryAbstractSolver* create() {
        return new T;
    }

    /**
     * @brief reg True if registration was successful
     */
    static bool reg;
    /**
     * @brief init Registers the class in the Solver Factory
     * @return True if successful
     */
    static bool init() {
        return BoundarySolverFactory::instance().registerType(T::name,
                BaseBoundarySolver::create);
    }
};

template <class T>
bool BaseBoundarySolver<T>::reg = BaseBoundarySolver<T>::init();
}
}  // end namespace

#endif  // BOUNDARYABSTRACTSOLVER_HPP
