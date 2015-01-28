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
#include <LbmLib/include/solver/FluidSolver/GuoZhengShi2002ForceModel.hpp>
#include <LbmLib/include/Direction.hpp>
#include <iostream>

namespace LbmLib {
namespace solver {

void GuoZhengShi2002ForceModel::computeForce(const double tau,
                                             const double rho,
                                             const Field<double> u,
                                             const Field<double> f,
                                             std::vector<double>& forcecontribution)
{
    /*
     * f_i = (1-1/(2*tau)) * w_i * [1/cs^2*(e_i-u)+1/cs^4*(e_i*u)e_i] * f
     * where e_i is the unit vector in direction i,
     */
    forcecontribution.clear();
    forcecontribution.resize(9);

    const double W0 = (1.0-0.5/tau) * 4.0/9.0;
    const double W1 = (1.0-0.5/tau) / 9.0;
    const double W2 = (1.0-0.5/tau) / 36.0;

    forcecontribution[T] = W0 * (- 3.0*u.x*f.x - 3.0*u.y*f.y);
    forcecontribution[E] = W1 * ((6.0*u.x + 3.0)*f.x - 3.0*u.y*f.y);
    forcecontribution[N] = W1 * (- 3.0*u.x*f.x + (6.0*u.y + 3.0)*f.y);
    forcecontribution[W] = W1 * ((6.0*u.x - 3.0)*f.x - 3.0*u.y*f.y);
    forcecontribution[S] = W1 * (- 3.0*u.x*f.x + (6.0*u.y - 3.0)*f.y);
    forcecontribution[NE] = W2 * ((6.0*u.x + 9.0*u.y + 3.0)*f.x + (9.0*u.x - 3.0*u.y + 3.0)*f.y);
    forcecontribution[NW] = W2 * ((6.0*u.x - 9.0*u.y - 3.0)*f.x + (-9.0*u.x + 6.0*u.y + 3.0)*f.y);
    forcecontribution[SW] = W2 * ((6.0*u.x + 9.0*u.y - 3.0)*f.x + (9.0*u.x + 6.0*u.y - 3.0)*f.y);
    forcecontribution[SE] = W2 * ((6.0*u.x - 9.0*u.y + 3.0)*f.x + (-9.0*u.x + 6.0*u.y - 3.0)*f.y);
}

void GuoZhengShi2002ForceModel::computeVelocity(const double rho,
                                                const std::array<double, 9> &fi,
                                                const Field<double> f,
                                                Field<double>& u)
{
    const double rhoI = 1.0/rho;
    u.x = (fi[E]
           + fi[NE]
           + fi[SE]
           - (fi[NW]
              + fi[W]
              + fi[SW])
           + 0.5*f.x) * rhoI;

    u.y = (fi[NE]
           + fi[N]
           + fi[NW]
           - (fi[SW]
              + fi[S]
              + fi[SE])
           + 0.5*f.y) * rhoI;
}

}
}


