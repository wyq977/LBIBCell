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
#ifndef GEOMETRYREPORTER_H
#define GEOMETRYREPORTER_H

#include <LbmLib/include/reportHandler/AbstractReportFunctor.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>

namespace LbmLib {
namespace nodes {
class GeometryNode;
}
namespace reportHandler {

/**
 * @brief This class reports current GeometryNodes.
 * Order of dumping: ID XPos YPos XVelocity YVelocity
 */
struct GeometryReporter : public AbstractReportFunctor {
    /**
     * @brief VelocityReporter constructor
     * @param GeometryNodes A map containing the GeometryNodes.
     * @param filename The filename where the dump is written to
     */
    GeometryReporter(
            const std::map<unsigned int, std::shared_ptr<LbmLib::nodes::GeometryNode> >& GeometryNodes,
            const std::string& filename) : AbstractReportFunctor(filename),
                                           GeometryNodes_(GeometryNodes) {}

    /**
     * @brief operator() writes the report
     * @param time The time step
     */
    virtual void operator()(unsigned int time) const;

 private:
    /**
     * @brief GeometryNodes_ Reference to the map containing GeometryNodes.
     */
    const std::map<unsigned int, std::shared_ptr<LbmLib::nodes::GeometryNode> >& GeometryNodes_;
};
}
}  // end namespace
#endif  // GEOMETRYREPORTER_H

