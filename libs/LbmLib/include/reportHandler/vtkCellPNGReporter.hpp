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
#ifndef VTKCELLPNGREPORTER_HPP
#define VTKCELLPNGREPORTER_HPP

#include <LbmLib/include/reportHandler/AbstractReportFunctor.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>

#include <vector>
#include <string>
#include <map>

namespace LbmLib {
namespace nodes {
class PhysicalNode;
class GeometryNode;
}
namespace geometry {
class Connection;
}
namespace solver {
class ForceSolver;
class AbstractForceStruct;
typedef std::shared_ptr< AbstractForceStruct> shptr_forcestruct;
typedef std::vector< shptr_forcestruct > vec_shptr_forcestruct;
typedef std::map< unsigned int, vec_shptr_forcestruct > map_forcestruct;
}

namespace reportHandler {
/**
 * @brief The vtkCellReporter class This class dumps the cell boundaries, cell attributes and cell forces in .vtm vtk format
 */
class vtkCellPNGReporter : public AbstractReportFunctor {
 public:
    /**
     * @brief vtkCellReporter The constructor.
     * @param connections The connections between the geomtry nodes.
     * @param filename The filename where the dump is written to.
     */
    vtkCellPNGReporter(
            const std::vector<std::shared_ptr<geometry::Connection> >& connections,
            const std::string& filename)
        : connections_(connections),
          AbstractReportFunctor(filename) {}

    /**
     * @brief operator() Writes the report
     * @param time The time step
     */
    virtual void operator()(unsigned int time) const;

 private:
    /**
     * @brief geoHandler_ Reference to the geometry handler
     */
    const std::vector<std::shared_ptr<geometry::Connection> >& connections_;
};
}
}  // end namespace
#endif  // VTKCELLPNGREPORTER_HPP
