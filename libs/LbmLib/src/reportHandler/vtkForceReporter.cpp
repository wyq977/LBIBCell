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
#include <LbmLib/include/reportHandler/vtkForceReporter.hpp>
#include <UtilLib/include/Exception.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/ForceSolver.hpp>
#include <LbmLib/include/solver/AbstractForceStruct.hpp>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkLine.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedIntArray.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkXMLMultiBlockDataWriter.h>
#include <vtkXMLMultiBlockDataReader.h>

#include <sys/stat.h>
#include <sstream>
#include <fstream>
#include <iomanip>

namespace LbmLib {
namespace reportHandler {
void vtkForceReporter::operator()(unsigned int time) const {
    std::stringstream filename;
    filename << filename_ << "_" << time << ".vtm";

    vtkSmartPointer<vtkMultiBlockDataSet> multiBDS =
            vtkSmartPointer<vtkMultiBlockDataSet>::New (); // multiblock container
    vtkSmartPointer<vtkXMLMultiBlockDataReader> reader =
            vtkSmartPointer<vtkXMLMultiBlockDataReader>::New(); // reader
    vtkSmartPointer<vtkXMLMultiBlockDataWriter> writer =
            vtkSmartPointer<vtkXMLMultiBlockDataWriter>::New(); // writer
    vtkSmartPointer<vtkPolyData> linePolyData =
            vtkSmartPointer<vtkPolyData>::New(); // line container
    vtkSmartPointer<vtkPoints> forcepoints =
            vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines =
            vtkSmartPointer<vtkCellArray>::New();
    std::vector<vtkSmartPointer<vtkLine> > linevector;
    vtkSmartPointer<vtkUnsignedIntArray> forcetype =
            vtkSmartPointer<vtkUnsignedIntArray>::New();
    forcetype->SetNumberOfComponents(1);
    forcetype->SetName("ForceType");

    LbmLib::solver::map_forcestruct forcemap = this->forcesolver_.getAllForces();
    unsigned int pointcounter = 0;

    for (auto iterator=forcemap.begin();
         iterator != forcemap.end();
         iterator++) {
        for (int k=0; k<iterator->second.size();++k) {
            if (iterator->second[k]->getPartnerGeometryNode() != 0) { // only if there is a line to draw
                forcepoints->InsertNextPoint(this->geometryNodes_.at(iterator->first)->getXPos(),
                                             this->geometryNodes_.at(iterator->first)->getYPos(),
                                             0.0);
                forcepoints->InsertNextPoint(this->geometryNodes_.at(iterator->second[k]->getPartnerGeometryNode())->getXPos(),
                                             this->geometryNodes_.at(iterator->second[k]->getPartnerGeometryNode())->getYPos(),
                                             0.0);
                forcetype->InsertNextValue(iterator->second[k]->getType()); // force type
                linevector.push_back(vtkSmartPointer<vtkLine>::New()); // a new line
                linevector.back()->GetPointIds()->SetId(0,pointcounter); // first point of the line
                linevector.back()->GetPointIds()->SetId(1,pointcounter+1); // second point of the line
                pointcounter += 2; // increment because two points have been added
            }
        }
    }

    for (int k=0; k<linevector.size(); ++k) {
        lines->InsertNextCell(linevector[k]);
    }

    //add the points and lines to the polydata
    linePolyData->SetPoints(forcepoints);
    linePolyData->SetLines(lines);
    linePolyData->GetCellData()->AddArray(forcetype);

    // assembly of the multiblock container:
    reader->SetFileName(filename.str().c_str());
    struct stat buffer;
    if (stat(filename.str().c_str(), &buffer) == 0) { // check if file already existing
        reader->Update(); // if yes, load its content
        multiBDS->ShallowCopy(reader->GetOutput());
    }

    multiBDS->SetBlock(multiBDS->GetNumberOfBlocks(),linePolyData); // add the force lines

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
