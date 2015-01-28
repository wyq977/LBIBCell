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
#include <LbmLib/include/solver/FluidSolver/BaseForceModel.hpp>
#include <vector>

#ifndef LUO1993FORCEMODEL_HPP
#define LUO1993FORCEMODEL_HPP

namespace LbmLib {
namespace solver {

/**
 * @brief The Luo1993ForceModel class implements the force model presented in Luo, Lattice-Gas Automata
 * and Lattice Boltzmann Equations for Two-Dimensional Hydrodynamics, Georgia Institute of Technology, 1993;
 * discussed in Mohamad, Kuzmin, A critical evaluation of force term in lattice Boltzmann method,
 * natural convection problem, Int. J. of Heat and Mass Tansfer, 2010
 */
class Luo1993ForceModel : private AbstractForceModel {
protected:
    /**
     * @brief computeForce
     * @param tau The relaxation time.
     * @param rho The local fluid density.
     * @param u The local fluid velocity.
     * @param f The local force.
     * @param forcecontribution The vector where the force contributions are stored and returned.
     */
    virtual void computeForce(const double tau,
                              const double rho,
                              const Field<double> u,
                              const Field<double> f,
                              std::vector<double>& forcecontribution);

    /**
     * @brief computeVelocity since this operation depends on the force model.
     * @param rho The local fluid density.
     * @param fi The reference to the array of 9 distribution functions.
     * @param f The local force field.
     * @param u The reference to the velocity vector where the result will be stored.
     */
    virtual void computeVelocity(const double rho,
                                 const std::array<double, 9>& fi,
                                 const Field<double> f,
                                 Field<double>& u);

};

}
}

#endif // LUO1993FORCEMODEL_HPP
