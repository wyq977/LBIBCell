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
#include <LbmLib/include/reportHandler/vtkCDEReporter.hpp>
#include <UtilLib/include/Exception.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>

#include <vtkSmartPointer.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#include <vtkMultiBlockDataSet.h>
#include <vtkXMLMultiBlockDataWriter.h>
#include <vtkXMLMultiBlockDataReader.h>
#include <vtkStructuredPoints.h>

#include <sys/stat.h>
#include <sstream>
#include <fstream>
#include <iomanip>

namespace LbmLib {
namespace reportHandler {
void vtkCDEReporter::operator()(unsigned int time) const {
    std::stringstream filename;
    filename << filename_ << "_" << time << ".vtm";

    std::vector<solver::CDEAbstractSolver*> cdesolvers;
    std::map<std::string,vtkSmartPointer<vtkDoubleArray> > cdesolvermap;
    vtkSmartPointer<vtkMultiBlockDataSet> multiBDS =
            vtkSmartPointer<vtkMultiBlockDataSet>::New ();
    vtkSmartPointer<vtkXMLMultiBlockDataReader> reader =
            vtkSmartPointer<vtkXMLMultiBlockDataReader>::New();
    vtkSmartPointer<vtkXMLMultiBlockDataWriter> writer =
            vtkSmartPointer<vtkXMLMultiBlockDataWriter>::New();
    vtkSmartPointer<vtkStructuredPoints> uniformGrid =
            vtkSmartPointer<vtkStructuredPoints>::New(); // the container which stores all fields

    cdesolvers = this->physicalNodes_[0][0]->getCDESolvers();
    unsigned int numcdesolvers = cdesolvers.size(); // number of cde solvers. consistency check later.
    for (auto k=0; k<numcdesolvers; ++k) {
        cdesolvermap[cdesolvers[k]->getName()] = vtkSmartPointer<vtkDoubleArray>::New();
        cdesolvermap[cdesolvers[k]->getName()]->SetNumberOfComponents(1);
        cdesolvermap[cdesolvers[k]->getName()]->SetName(cdesolvers[k]->getName().c_str());
    }

    unsigned int xLatticeSize = (unsigned int)(this->physicalNodes_[0].size()/this->cdecoarseningfactor_);
    unsigned int yLatticeSize = (unsigned int)(this->physicalNodes_.size()/this->cdecoarseningfactor_);


    uniformGrid->SetDimensions(xLatticeSize,yLatticeSize,1);
    uniformGrid->SetOrigin(0,0,0);
    uniformGrid->SetSpacing(this->cdecoarseningfactor_,
                            this->cdecoarseningfactor_,
                            this->cdecoarseningfactor_); // adjust spacing when coarsening

    for (auto ity=0; ity<this->physicalNodes_.size(); ity+=this->cdecoarseningfactor_) { // loop y
        for (auto itx=0; itx<this->physicalNodes_[0].size(); itx+=this->cdecoarseningfactor_) { // loop x
            cdesolvers =this->physicalNodes_[ity][itx]->getCDESolvers(); // get all the solvers from node [y][x]
            for (auto k=0; k<numcdesolvers; ++k) { // loop all cdesolvers
                cdesolvermap[cdesolvers[k]->getName()]->InsertNextValue(cdesolvers[k]->getC()); // add the concentration
            }
        }
    }

    for (auto it : cdesolvermap) { // add all the fields to the uniform grid:
        uniformGrid->GetPointData()->AddArray(it.second);
    }

    reader->SetFileName(filename.str().c_str());
    struct stat buffer;
    if (stat(filename.str().c_str(), &buffer) == 0) { // check if already existing
        reader->Update(); // if yes, load its content
        multiBDS->ShallowCopy(reader->GetOutput());
    }

    multiBDS->SetBlock(multiBDS->GetNumberOfBlocks(),uniformGrid);

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
