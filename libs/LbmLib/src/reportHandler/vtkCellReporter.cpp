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
#include <LbmLib/include/reportHandler/vtkCellReporter.hpp>
#include <UtilLib/include/Exception.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkDoubleArray.h>
#include <vtkStringArray.h>
#include <vtkPointData.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkXMLMultiBlockDataWriter.h>
#include <vtkXMLMultiBlockDataReader.h>

#include <sys/stat.h>
#include <sstream>
#include <fstream>
#include <iomanip>

namespace LbmLib {
namespace reportHandler {
void vtkCellReporter::operator()(unsigned int time) const {
    std::stringstream filename;
    filename << filename_ << "_" << time << ".vtm";

    vtkSmartPointer<vtkPoints> polygonpoints =
            vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polygons =
            vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyData> polygonPolyData =
            vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkMultiBlockDataSet> multiBDS =
            vtkSmartPointer<vtkMultiBlockDataSet>::New ();
    vtkSmartPointer<vtkXMLMultiBlockDataReader> reader =
            vtkSmartPointer<vtkXMLMultiBlockDataReader>::New();
    vtkSmartPointer<vtkXMLMultiBlockDataWriter> writer =
            vtkSmartPointer<vtkXMLMultiBlockDataWriter>::New();
    std::vector<vtkSmartPointer<vtkPolygon> > polygonvector;
    std::map<unsigned int,std::vector<std::shared_ptr<LbmLib::geometry::Connection> > > celldefinition;
    std::shared_ptr<LbmLib::geometry::Connection> startC = nullptr;
    std::shared_ptr<LbmLib::geometry::Connection> tempC = nullptr;
    vtkSmartPointer<vtkDoubleArray> domainidentifier =
            vtkSmartPointer<vtkDoubleArray>::New();
    domainidentifier->SetName("domain_identifier");
    vtkSmartPointer<vtkDoubleArray> celltype =
            vtkSmartPointer<vtkDoubleArray>::New();
    celltype->SetName("cell_type");

    vtkSmartPointer<vtkStringArray> stringCDESolverArray =
      vtkSmartPointer<vtkStringArray>::New();

    stringCDESolverArray->SetNumberOfComponents(1);
    stringCDESolverArray->SetName("boundarydescriptors");

    std::stringstream boundarydescriptor;

    int globalpointcounter = 0;
    int polygonpointcounter;
    int polygoncounter = 0;
    unsigned int temp_celltype;

    for (auto it : this->connections_) { // create a cell definition map:
        celldefinition[(*it).getDomainIdentifier()].push_back(it);
    }

    for (auto it : celldefinition) { // loop the cells
        startC = it.second[0];
        tempC = startC;
        polygonpointcounter = 0;
        polygonvector.push_back(vtkSmartPointer<vtkPolygon>::New());

        do {
            polygonpoints->InsertNextPoint(tempC->getGeometryNodes().second->getXPos(),
                                    tempC->getGeometryNodes().second->getYPos(),
                                    0.0); // add the points
            domainidentifier->InsertNextValue(tempC->getDomainIdentifier()); // add the domain identifier

            if ( this->cellTypeTrackerMap_.find(tempC->getDomainIdentifier()) == this->cellTypeTrackerMap_.end() ) {
                lbm_fail("domainIdentifier not found in the cellTrackerMap.");
            } else {
                temp_celltype = this->cellTypeTrackerMap_.at(tempC->getDomainIdentifier()); // get the celltype
            }

            celltype->InsertNextValue(temp_celltype); // add the cell type

            boundarydescriptor.str(""); // clear string
            boundarydescriptor << "\t"; // to avoid segfaults when there is no other content
            for (const auto& bSolver : tempC->getBoundaryConditionDescriptor()) { // assmble a string with the boundary descriptor
                for (const auto& cdeSolver : bSolver.second) {
                    boundarydescriptor << bSolver.first << '\t' << cdeSolver << '\t';
                }
            }

            stringCDESolverArray->InsertNextValue(boundarydescriptor.str().c_str());
            polygonvector.back()->GetPointIds()->InsertId(polygonpointcounter, globalpointcounter);

            tempC = tempC->getGeometryNodes().second->getConnection<1>(); // go to next connection
            globalpointcounter++;
            polygonpointcounter++;
        } while(tempC != startC); // do until reaching beginning of the polygon
        polygoncounter++;
    }

    for (auto k : polygonvector) { // add the polygons to a list of polygons
        polygons->InsertNextCell(k);
    }

    polygonPolyData->SetPoints(polygonpoints); // add the points to the polydata container
    polygonPolyData->SetPolys(polygons);
    polygonPolyData->GetPointData()->AddArray(domainidentifier); // add the attributes
    polygonPolyData->GetPointData()->AddArray(celltype); // add the attributes
    polygonPolyData->GetPointData()->AddArray(stringCDESolverArray); // add the boundarydescriptor strings

    reader->SetFileName(filename.str().c_str());
    struct stat buffer;
    if (stat(filename.str().c_str(), &buffer) == 0) { // check if file already existing

        reader->Update(); // if yes, load its content
        multiBDS->DeepCopy(reader->GetOutput());
    }

    multiBDS->SetBlock(multiBDS->GetNumberOfBlocks(),polygonPolyData);

    writer->SetFileName(filename.str().c_str());
#if VTK_MAJOR_VERSION <= 5
    writer->SetInput(multiBDS);
#else
    writer->SetInputData(multiBDS);
#endif
    //writer->SetCompressorTypeToNone();
    writer->SetDataModeToAscii();
    //writer->SetDataModeToBinary();
    writer->Write();
}
}
}  // end namespace
