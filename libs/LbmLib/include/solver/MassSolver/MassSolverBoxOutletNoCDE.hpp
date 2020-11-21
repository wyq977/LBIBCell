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
#ifndef MASSSOLVERBOXOUTLETNOCDE_HPP
#define MASSSOLVERBOXOUTLETNOCDE_HPP

#include <LbmLib/include/solver/MassSolver/MassAbstractSolver.hpp>
#include <string>
#include <vector>

namespace LbmLib {
namespace solver {
/**
 * @brief this solver introduces a constant local mass source to all domains with domainID!=0.
 * Furthermore, it defines a mass ink at all four box boundary
 * x=0, x=LX-1, y=0, y=LY-1
 * Based on MassSolverBoxOutlet but remove the CDE solver 
 * Boundary condition for CDESolver should be defined elsewhere
 */
class MassSolverBoxOutletNoCDE : public BaseMassSolver<MassSolverBoxOutletNoCDE> {
 public:
    /**
     * @brief The isopressure outflow boundary condition for the domain.
     * @param fluidGrid the fluid nodes
     */
    virtual void calculateMass(
            const std::vector<std::vector<nodes::PhysicalNode*> >& fluidGrid);

 private:
    /**
     * @brief Define the constructor as private member to avoid initialisation of this class.
     */
    friend class BaseMassSolver;
    MassSolverBoxOutletNoCDE();
    /**
     * @brief name The name of this solver.
     */
    static const std::string name;
};
}
}  // end namespace

#endif  // MASSSOLVERBOXOUTLETNOCDE_HPP


