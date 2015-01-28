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
#ifndef CDEABSTRACTSOLVER_HPP
#define CDEABSTRACTSOLVER_HPP
#include <LbmLib/include/solver/AbstractSolver.hpp>
#include <LbmLib/include/solver/CDESolver/CDESolverFactory.hpp>
#include <array>
#include <string>
#include <iostream>

namespace LbmLib {
namespace nodes {
class PhysicalNode;
}
namespace solver {
/**
 * @brief The abstract base class for all CDESolvers
 * @attention Do not inherit from this class! Use class \c BaseCDESolver instead, as the latter class will register the class automatically
 */
class CDEAbstractSolver : public AbstractSolver {
 public:
    /**
     * @brief ~CDEAbstractSolver virtual Destructor
     */
    virtual ~CDEAbstractSolver() {}

    /**
     * @brief initCDESolver This connects the Solver with an physical node this should be
     * executed before using this class
     * @param physicalNode The node this solver is connected to.
     * @param id The id of this solver
     */
    void initCDESolver(
            const nodes::PhysicalNode* physicalNode,
            size_t id);

    /**
     * @brief calculateEquilibrium calculates the equilibirum for direction dir
     * @param dir the direction
     * @return the Equilibrium
     */
    virtual double calculateEquilibrium(const Direction& dir) = 0;

    /**
     * @brief getC Calculates the concentration on this node
     * @return The concentration on this node
     */
    virtual double getC() const = 0;

    /**
     * @brief reinitialise this solver iff the corresponding physical node has changed domain identifier
     */
    virtual void reinitialise() = 0;

    /**
     * @brief Get the name of the solver
     * @return the name of the solver
     */
    virtual std::string getName() = 0;

    /**
     * @brief return the id of this solver
     * @return the id of this solver
     */
    size_t getId();

 protected:
    /**
     * @brief CDEAbstractSolver protected Constructor only use create method for instantiation
     */
    CDEAbstractSolver();
    /**
     * @brief physicalNode_ The physical Node which owns this solver
     */
    const nodes::PhysicalNode* physicalNode_;

    /**
     * @brief solverID_ The ID of the solver instance. Coincides with the index in the vector PhysicalNode::cdeSolvers_ (which stores all CDE solvers).
     */
    size_t solverID_;

 private:
};

/**
 * @brief The Base class for all CDESolver implementations
 * This classes uses the recursive template idiom to automatically register child classes in the factory.
 * To implement a solver, inherit from this class and provide the same class as a template argument.
 * Additionally, the class should provide a static member to store a unique name for the class.
 * Preferentially, declare the constructor and the static member name private and make the
 * BaseCDESolver a friend class of this.
 * An Example for a CDESolver:
 **\code
 * class CDESolverD2Q5 : public BaseCDESolver<CDESolverD2Q5>{
 *
 * private:
 *
 *   friend class BaseCDESolver;
 *
 *   CDESolverD2Q5();
 *
 *   static const std::string name; // the name of the CDEsolver, e.g. CDESolverD2Q5
 *
 * }
 **\endcode
 */
template <class T>
struct BaseCDESolver : CDEAbstractSolver {
 public:
    /**
     * @brief getName
     * @return  returns the name of the solver.
     */
    std::string getName() {
        return T::name;
    }

 protected:
    /**
     * @brief BaseCDESolver The Constructor which enforces that the specialisation is done
     */
    BaseCDESolver() {
        reg = reg;  // force specialization
    }

    /**
     * @brief ~BaseCDESolver virtual Destructor
     */
    virtual ~BaseCDESolver() {}

    /**
     * @brief Create static constructor.
     * A pointer to this function along with the
     * name of the algorithm is registered with the Solver Factory.
     * @return Pointer to a new \c AbstractCDESolver instance. Responsibility for
     *         allocated memory lies with the caller.
     */
    static CDEAbstractSolver* create() {
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
        return CDESolverFactory::instance().registerType(T::name,
                BaseCDESolver::create);
    }
};

template <class T>
bool BaseCDESolver<T>::reg = BaseCDESolver<T>::init();
}
}  // end namespace

#endif  // CDEABSTRACTSOLVER_HPP
