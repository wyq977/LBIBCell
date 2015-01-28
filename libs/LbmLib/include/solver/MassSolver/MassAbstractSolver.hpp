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
#ifndef MASSABSTRACTSOLVER_HPP
#define MASSABSTRACTSOLVER_HPP
#include <LbmLib/include/solver/MassSolver/MassSolverFactory.hpp>
#include <array>
#include <string>
#include <vector>

namespace LbmLib {
namespace nodes {
class PhysicalNode;
}
namespace solver {
/**
 * @brief The MassAbstractSolver class The abstract base class for all Mass Solvers
 * @attention Do not inherit from this class. Use class \c BaseMassSolver instead, as this will then register the class automatically.
 */
class MassAbstractSolver {
 public:
    /**
     * @brief ~MassAbstractSolver virtual Destructor
     */
    virtual ~MassAbstractSolver() {}

    /**
     * @brief the virtual method which calculates the masses on the Geometry Nodes
     * @param fluidGrid The fluid grid
     */
    virtual void calculateMass(
            const std::vector<std::vector<nodes::PhysicalNode*> >& fluidGrid) =
        0;

    /**
     * @brief Get the name of the solver
     * @return the name of the solver
     */
    virtual std::string getName() = 0;

 protected:
    /**
     * @brief protected Constructor only use create method for instantiation
     */
    MassAbstractSolver() {}
};

/**
 * @brief The Base class for all Mass Solvers implementations
 * This classes uses the recursive template idiom to automatically register child classes in the factory.
 * To implement a solver inherit from this class and provide the same class as a template argument.
 * Additionally, the class should provide a static member with the name which stores a
 * unique name for the class.
 * Preferentially, declare the constructor and the static member name private and make the
 * BaseForceSolver a friend class of this.
 * look at /c BaseCDESolver for an example
 */
template <class T>
struct BaseMassSolver : MassAbstractSolver {
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
     * @brief BaseMassSolver The Constructor which enforces that the specialisation is done
     */
    BaseMassSolver() {
        reg = reg;  // force specialization
    }

    /**
     * @brief ~BaseMassSolver virtual Destructor
     */
    virtual ~BaseMassSolver() {}

    /**
     * @brief create Static constructor.
     * A pointer to this function along with the
     * name of the algorithm is registered with the Solver Factory.
     * @return Pointer to a new \c MassAbstractSolver instance. Responsibility for
     *         allocated memory lies with the caller.
     */
    static MassAbstractSolver* create() {
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
        return MassSolverFactory::instance().registerType(T::name,
                BaseMassSolver::create);
    }
};

template <class T>
bool BaseMassSolver<T>::reg = BaseMassSolver<T>::init();
}
}  // end namespace

#endif  // MASSABSTRACTSOLVER_HPP
