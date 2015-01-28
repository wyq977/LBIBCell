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
#ifndef VELOCITYREPORTER_HPP
#define VELOCITYREPORTER_HPP

#include <LbmLib/include/reportHandler/AbstractReportFunctor.hpp>
#include <vector>
#include <string>
namespace LbmLib {
namespace nodes {
class PhysicalNode;
}
namespace reportHandler {
/**
 * @brief The DensityReporter class This class reports the velocity of the fluid solver.
 * The dump look as if: xPos yPos vx vy
 */
struct VelocityReporter : public AbstractReportFunctor {
    /**
     * @brief VelocityReporter constructor
     * @param physicalNodes The physical Nodes
     * @param filename The filename where the dump is written to
     */
    VelocityReporter(
            const std::vector<std::vector<nodes::PhysicalNode*> >& physicalNodes,
            const std::string& filename) : AbstractReportFunctor(filename),
                                           physicalNodes_(physicalNodes) {}

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
};
}
}  // end namespace
#endif  // VELOCITYREPORTER_HPP
