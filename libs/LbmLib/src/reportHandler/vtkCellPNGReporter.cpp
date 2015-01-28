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
#include <LbmLib/include/reportHandler/vtkCellPNGReporter.hpp>
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

#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkTriangleFilter.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkImageCanvasSource2D.h>
#include <vtkImageCast.h>

#include <sys/stat.h>
#include <sstream>
#include <fstream>
#include <iomanip>

namespace LbmLib {
namespace reportHandler {
void vtkCellPNGReporter::operator()(unsigned int time) const {
    std::stringstream filename;
    filename << filename_ << "_" << time << ".png";

    vtkSmartPointer<vtkPoints> polygonpoints =
            vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polygons =
            vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkPolyData> polygonPolyData =
            vtkSmartPointer<vtkPolyData>::New();
    vtkTriangleFilter *tri=
            vtkTriangleFilter::New();
    vtkPolyDataMapper *map =
            vtkPolyDataMapper::New();
    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    std::vector<vtkSmartPointer<vtkPolygon> > polygonvector;
    vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =
            vtkSmartPointer<vtkWindowToImageFilter>::New();
    vtkSmartPointer<vtkPNGWriter> writer =
            vtkSmartPointer<vtkPNGWriter>::New();
    std::map<unsigned int,std::vector<std::shared_ptr<LbmLib::geometry::Connection> > > celldefinition;
    std::shared_ptr<LbmLib::geometry::Connection> startC = nullptr;
    std::shared_ptr<LbmLib::geometry::Connection> tempC = nullptr;

    int globalpointcounter = 0;
    int polygonpointcounter;
    int polygoncounter = 0;

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

#if VTK_MAJOR_VERSION <= 5
    tri->SetInput(polygonPolyData); //The output of the triangle filter will be a triangulation of the inputPolyData
    map->SetInput(tri->GetOutput());
#else
    tri->SetInputData(polygonPolyData); //The output of the triangle filter will be a triangulation of the inputPolyData
    map->SetInputData(tri->GetOutput());
#endif

    actor->SetMapper(map);

    renderWindow->SetOffScreenRendering( 1 );
    renderWindow->AddRenderer(renderer);


    renderer->AddActor(actor);
    renderer->SetBackground(0,0,0);
    renderWindow->Render();

    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->SetMagnification(3); //set the resolution of the output image (3 times the current resolution of vtk render window)
    //windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
    windowToImageFilter->ReadFrontBufferOff(); // read from the back buffer
    windowToImageFilter->Update();

    writer->SetFileName(filename.str().c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
}
}
}  // end namespace
