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
#include <LbmLib/include/solver/FluidSolver/Luo1993ForceModel.hpp>
#include <LbmLib/include/Direction.hpp>
#include <iostream>

namespace LbmLib {
namespace solver {

void Luo1993ForceModel::computeForce(const double tau,
                                     const double rho,
                                     const Field<double> u,
                                     const Field<double> f,
                                     std::vector<double>& forcecontribution)
{
    const double W1rho3 = rho * 3.0 / 9.0;
    const double W2rho3 = rho *3.0 / 36.0;

    forcecontribution.resize(9);

    forcecontribution[T] = 0.0;
    forcecontribution[E] = W1rho3 * f.x;
    forcecontribution[N] = W1rho3 * f.y;
    forcecontribution[W] = - W1rho3 * f.x;
    forcecontribution[S] = - W1rho3 * f.y;
    forcecontribution[NE] = W2rho3 * (f.x + f.y);
    forcecontribution[NW] = W2rho3 * (- f.x + f.y);
    forcecontribution[SW] = W2rho3 * (- f.x - f.y);
    forcecontribution[SE] = W2rho3 * (f.x - f.y);
}

void Luo1993ForceModel::computeVelocity(const double rho,
                                        const std::array<double, 9> &fi,
                                        const Field<double> f,
                                        Field<double> &u)
{
    const double rhoI = 1.0/rho;
    u.x = (fi[E]
           + fi[NE]
           + fi[SE]
           - (fi[NW]
              + fi[W]
              + fi[SW])) * rhoI;

    u.y = (fi[NE]
           + fi[N]
           + fi[NW]
           - (fi[SW]
              + fi[S]
              + fi[SE])) * rhoI;
}

}
}
