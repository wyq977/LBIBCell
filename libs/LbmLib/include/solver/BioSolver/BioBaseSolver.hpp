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
#ifndef BIOBASESOLVER_HPP
#define BIOBASESOLVER_HPP
#include <LbmLib/include/solver/BioSolver/BioSolverFactory.hpp>
#include <LbmLib/include/solver/BioSolver/BioAbstractSolver.hpp>
#include <array>
#include <string>
#include <vector>

namespace LbmLib {
namespace nodes {
class PhysicalNode;
}
namespace solver {
/**
 * @brief The Base class for all BioSolver implementations.
 * This classes uses the recursive template idiom to automatically register child classes in the factory.
 * To implement a solver inherit from this class and provide the same class as a template argument.
 * Additionally, the class must provide a static member with the name which stores a
 * unique name for the class.
 * Declare the constructor and the static member name private and make the
 * BioBaseSolver a friend class of this.
 * @todo look at ... for an example
 */
template <class T>
struct BioBaseSolver : BioAbstractSolver {
 public:
    /**
     * @brief getName
     * @return returns the name of the solver
     */
    std::string getName() {
        return T::name;
    }

 protected:
    /**
     * @brief The Constructor which enforces specialization.
     */
    BioBaseSolver() {
        reg = reg;  // force specialization
    }

    /**
     * @brief Virtual destructor.
     */
    virtual ~BioBaseSolver() {}

    /**
     * @brief Static constructor.
     * A pointer to this function along with the
     * name of the algorithm is registered in the Solver Factory.
     * @return Pointer to a new \c BioAbstractSolver instance. Responsibility for
     *         allocated memory lies with the caller.
     */
    static BioAbstractSolver* create() {
        return new T;
    }

    /**
     * @brief reg True if registration was successful.
     */
    static bool reg;

    /**
     * @brief init Registers the class in the Solver Factory
     * @return True if successful.
     */
    static bool init() {
        return BioSolverFactory::instance().registerType(T::name,
                BioBaseSolver::create);
    }
};

template <class T>
bool BioBaseSolver<T>::reg = BioBaseSolver<T>::init();
}
}  // end namespace

#endif  // BIOBASESOLVER_HPP

