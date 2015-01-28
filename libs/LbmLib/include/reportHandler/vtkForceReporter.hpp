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
#ifndef VTKFORCEREPORTER_HPP
#define VTKFORCEREPORTER_HPP

#include <LbmLib/include/reportHandler/AbstractReportFunctor.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>

#include <vector>
#include <string>

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
 * @brief The vtkForceReporter class This class dumps the cell forces in .vtm vtk format
 */
class vtkForceReporter : public AbstractReportFunctor {
 public:
    /**
     * @brief vtkForceReporter The constructor.
     * @param geometryNodes The geometry nodes.
     * @param forcesolver The force solver.
     * @param filename The filename to dump to.
     */
    vtkForceReporter(
            const std::map<unsigned int, std::shared_ptr<LbmLib::nodes::GeometryNode> >& geometryNodes,
            const solver::ForceSolver& forcesolver,
            const std::string& filename)
        : geometryNodes_(geometryNodes),
          forcesolver_(forcesolver),
          AbstractReportFunctor(filename) {}

    /**
     * @brief operator() Writes the report
     * @param time The time step
     */
    virtual void operator()(unsigned int time) const;

 private:
    /**
     * @brief forcesolver_ a reference to the force solver
     */
    const solver::ForceSolver& forcesolver_;
    /**
     * @brief GeometryNodes_ Reference to the map containing GeometryNodes.
     */
    const std::map<unsigned int, std::shared_ptr<LbmLib::nodes::GeometryNode> >& geometryNodes_;
};
}
}  // end namespace
#endif  // VTKFORCEREPORTER_HPP
