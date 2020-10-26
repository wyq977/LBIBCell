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
#include <LbmLib/include/geometry/Geometry.hpp>
#include <LbmLib/include/geometry/GeometryHandler.hpp>
#include <LbmLib/include/SimulationRunner.hpp>

#include <LbmLib/include/reportHandler/ReportHandler.hpp>
#include <LbmLib/include/reportHandler/vtkCellReporter.hpp>
#include <LbmLib/include/reportHandler/vtkForceReporter.hpp>
#include <LbmLib/include/reportHandler/vtkCDEReporter.hpp>
#include <LbmLib/include/reportHandler/vtkFluidReporter.hpp>
#include <LbmLib/include/reportHandler/vtkCellPNGReporter.hpp>

#include <iostream>
#include <string>
#include <fstream>
#include <sys/stat.h>

int main(
        int argc,
        char* argv[]) {

        // arg parsing
        std::string outfolder;
        std::string parameters;

        if (argc == 3) {
            outfolder = argv[1];
            parameters = argv[2];
        } else {
            std::cout << "Usage: " << argv[0]
                      << " output_dir parameters.txt"
                      << std::endl;
        return EXIT_FAILURE;
        }

        std::cout << std::string(80, '*') << std::endl
                  << "Output dir: " << outfolder << std::endl
                  << "Parameters: " << parameters << std::endl
                  << std::string(80, '*') << std::endl;

    try {
        // create output folder and clear content:
        std::string rm_files_cmd = "exec rm -r " + outfolder + "*";

        const int mk = mkdir(outfolder.c_str(), 0777);
        if (mk==1) {
            lbm_fail("Cannot create the output directory.");
        }
        auto ret = system(rm_files_cmd.c_str());

        std::stringstream fileName;

        //set up log facility:
        std::shared_ptr<std::ostream> pStream(new std::ofstream(outfolder+"log.txt"));
        if (!pStream) {
            lbm_fail("Cannot open the log file.");
        }
        UtilLib::Log::setStream(pStream);

        LbmLib::Parameters.loadGlobalSimulationParameters(parameters);
        LbmLib::Parameters.printParameters();

        //LbmLib::geometry::Geometry geo("config/tutorial_01_geometry.txt");
        LbmLib::geometry::Geometry geo("config/vtk/Cells_0.vtm");
        LbmLib::geometry::GeometryHandler geohandler(geo);

        LbmLib::reportHandler::ReportHandler reporter(
                LbmLib::Parameters.getReportSteps());

        LbmLib::SimulationRunner simRunner(geohandler, reporter);

        // vtk cell reporter:
        fileName.str("");
        fileName << outfolder+"Cells";
        reporter.registerReporter(std::unique_ptr < LbmLib::reportHandler::
                                  AbstractReportFunctor > (new LbmLib::reportHandler::
                                                           vtkCellReporter(
                                                               geo.getConnections(),
                                                               geohandler.getCellTypeTrackerMap(),
                                                               fileName.str()
                                                               )
                                                           )
                                  );

        // vtk force reporter:
        fileName.str("");
        fileName << outfolder+"Cells";
        reporter.registerReporter(std::unique_ptr < LbmLib::reportHandler::
                                  AbstractReportFunctor > (new LbmLib::reportHandler::
                                                           vtkForceReporter(
                                                               geo.getGeometryNodes(),
                                                               simRunner.getForceSolver(),
                                                               fileName.str()
                                                               )
                                                           )
                                  );

        // vtk fluid reporter:
        fileName.str("");
        fileName << outfolder+"Cells";
        reporter.registerReporter(std::unique_ptr < LbmLib::reportHandler::
                                  AbstractReportFunctor > (new LbmLib::reportHandler::
                                                           vtkFluidReporter(
                                                               geohandler.getPhysicalNodes(),
                                                               4, // cde coarseningfactor; 1=full resolution
                                                               fileName.str()
                                                               )
                                                           )
                                  );

        simRunner.initSolvers();
        simRunner.initForceSolver("config/force.txt");

        simRunner.addMassSolver("MassSolverBoxOutlet");
        // simRunner.addBioSolver("tutorial_01_BioSolverAreaRegulator");
        // simRunner.addBioSolver("tutorial_01_BioSolverMembraneTension");
        // simRunner.addBioSolver("tutorial_01_BioSolverCellJunction");
        // simRunner.addBioSolver("tutorial_01_BioSolverDifferentiation");
        // simRunner.addBioSolver("tutorial_01_BioSolverCellDivision");
        simRunner.addBioSolver("BioSolverAreaRegulator");
        simRunner.addBioSolver("BioSolverCellDivision");
        simRunner.addBioSolver("BioSolverCellJunction");
        simRunner.addBioSolver("BioSolverGrowth");
        simRunner.addBioSolver("BioSolverMembraneTension");

        simRunner.runSimulation();

    } catch(const std::exception& exp) {
        std::cout << exp.what() << std::endl;
    }

    return 0;
}
