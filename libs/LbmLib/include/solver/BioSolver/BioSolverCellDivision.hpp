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
#ifndef BIOSOLVERCELLDIVISION_HPP
#define BIOSOLVERCELLDIVISION_HPP

#include <LbmLib/include/solver/BioSolver/BioBaseSolver.hpp>
#include <string>
#include <vector>
#include <memory>

namespace LbmLib {
namespace geometry {
class Connection;
}
namespace solver {
namespace {
    /**
     * @brief The MinMaxStruct struct
     */
    struct MinMaxStruct {
      double xmin; ///< xmin of the cell
      double xmax; ///< xmax of the cell
      double ymin; ///< ymin of the cell
      double ymax; ///< ymax of the cell
      MinMaxStruct():xmin(0),xmax(0),ymin(0),ymax(0) { }
    } ;
}

/**
 * @brief BioSolver which divides cells.
 */
class BioSolverCellDivision : public BioBaseSolver<BioSolverCellDivision> {
 public:
    /**
     * @brief Applies biological processes.
     * @param geometryhandler The GeometryHandler.
     * @param forcesolver The ForceSolver.
     */
    virtual void applyBioProcess(
            geometry::GeometryHandler& geometryhandler,
            solver::ForceSolver& forcesolver
            );

 private:
    /**
     * @brief Make the constructor private to avoid instantiation of this class.
     */
    friend class BioBaseSolver;
    BioSolverCellDivision();

    /**
     * @brief divideCell
     * @param geometryhandler The GeometryHandler.
     * @param domainidentifier The identifier of the cell which will be divided
     */
    void divideCell(geometry::GeometryHandler& geometryhandler, unsigned int domainidentifier);

    /**
     * @brief Get the two *Connection*s which intersect with the line which is perpendicular to the axis defined by the furthest point pair.
     * @param domainidentifier The domainIdentifier of the cell which is analyzed.
     * @return The two *Connection*s which will be deleted in the division process later.
     */
    const std::vector<std::shared_ptr<LbmLib::geometry::Connection> >
    getTwoConnectionsLongestAxis(unsigned int domainidentifier);

    /**
     * @brief Get the two *Connection*s which intersect with a line with random orientation
     * @param domainidentifier The domainIdentifier of the cell which is analyzed.
     * @return The two *Connection*s which will be deleted in the division process later.
     */
    const std::vector<std::shared_ptr<LbmLib::geometry::Connection> >
    getTwoConnectionsRandomDirection(unsigned int domainidentifier);

    /**
     * @brief Updates this->cellDefinition_.
     * @param geometryhandler The GeometryHandler.
     */
    void updateCellDefinition(geometry::GeometryHandler& geometryhandler);

    /**
     * @brief cellDefinition_ The *Connection*s are in a map. First dimension: cell identity. Second dimension: the *Connection*s defining that cell.
     */
    std::map<unsigned int,std::vector<std::shared_ptr<LbmLib::geometry::Connection> > > cellDefinition_;

    /**
     * @brief name The name of this solver
     */
    static const std::string name;
};
}
}  // end namespace

#endif  // BIOSOLVERCELLDIVISION_HPP


