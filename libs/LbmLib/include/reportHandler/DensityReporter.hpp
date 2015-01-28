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
#ifndef DENSITYREPORTER_HPP
#define DENSITYREPORTER_HPP
#include <LbmLib/include/reportHandler/AbstractReportFunctor.hpp>
#include <vector>
#include <string>
namespace LbmLib {
namespace nodes {
class PhysicalNode;
}
namespace reportHandler {
/**
 * @brief The DensityReporter class This class reports the density of a solver.
 * The dump look as if: xPos yPos Density
 */
class DensityReporter : public AbstractReportFunctor {
 public:
    /**
     * @brief DensityReporter The constructor
     * @param physicalNodes The physical nodes
     * @param filename The filename where the dump is written to
     * @param solverName The name of the solver. If omitted, the mass density of the fluid solver is taken.
     */
    DensityReporter(
            const std::vector<std::vector<nodes::PhysicalNode*> >&
            physicalNodes,
            const std::string& filename,
            const std::string& solverName = "")
        : AbstractReportFunctor(filename),
          physicalNodes_(physicalNodes),
          solverName_(solverName) {}

    /**
     * @brief operator() Writes the report
     * @param time The time step
     */
    virtual void operator()(unsigned int time) const;

 private:
    /**
     * @brief physicalNodes_ Reference to the physical Nodes
     */
    const std::vector<std::vector<nodes::PhysicalNode*> >& physicalNodes_;
    /**
     * @brief solverName_ The name of the solver where the distribution is dumped
     */
    const std::string solverName_;
};
}
}  // end namespace
#endif  // DENSITYREPORTER_HPP
