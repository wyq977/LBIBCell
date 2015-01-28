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
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <UtilLib/include/Exception.hpp>
#include <omp.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace LbmLib {
GlobalSimulationParameters_::GlobalSimulationParameters_() {}

void GlobalSimulationParameters_::loadGlobalSimulationParameters(
        const std::string& fileName) {
    std::ifstream fileStream;
    fileStream.open(fileName);
    if (fileStream.is_open()) {
        std::string line;
        while (std::getline(fileStream, line)) {
            std::stringstream lineStream(line);
            if (line.at(0) != '#') {
                std::string identifier;
                lineStream >> identifier;
                if (identifier == "CurrentIterations:") {
                    lineStream >> currentIteration_;
                } else if (identifier == "Iterations:") {
                    lineStream >> iterations_;
                } else if (identifier == "ReportStep:") {
                    lineStream >> reportSteps_;
                } else if (identifier == "SizeX:") {
                    lineStream >> sizeX_;
                } else if (identifier == "SizeY:") {
                    lineStream >> sizeY_;
                } else if (identifier == "tauFluid:") {
                    lineStream >>  tauFluid_;
                } else if (identifier == "CDESolvers:") {
                    std::string tempName;
                    double tempTau;
                    while (lineStream >> tempName >> tempTau) {
                        cdeSolvers_[tempName] = tempTau;
                    }
                } else {  throw UtilLib::Exception(
                                  "An unknown parameter was passed in the global Parameter file");
                }
            }
        }
    } else {
        lbm_fail("Cannot find the parameter file.");
    }
    fileStream.close();
}

void GlobalSimulationParameters_::writeGlobalSimulationParameters(
        const std::string& fileName) const {
    std::ofstream fileStream;
    fileStream.open(fileName);
    if (fileStream.is_open()) {
        fileStream << "#Global Parameter file\n";
        fileStream << "CurrentIterations:" << '\t' << currentIteration_ << "\n";
        fileStream << "Iterations:" << '\t' << iterations_ << "\n";
        fileStream << "ReportStep:" << '\t' << reportSteps_ << "\n";
        fileStream << "SizeX:" << '\t' << sizeX_ << "\n";
        fileStream << "SizeY:" << '\t' << sizeY_ << "\n";
        fileStream << "tauFluid:" << '\t' << tauFluid_ << "\n";
        fileStream << "CDESolvers:";
        for (const auto& solver : cdeSolvers_) {
            fileStream << '\t' << solver.first << '\t' << solver.second;
        }
        fileStream << "\n";
    } else {
        throw UtilLib::Exception(
                "Problem to open the output file for the force solver");
    }

    fileStream.close();
}

void GlobalSimulationParameters_::printParameters() const {
    const int num_threads(omp_get_max_threads());
    omp_set_num_threads(num_threads);
    std::cout << std::endl;
    std::cout << std::left << std::setfill('*')  << std::setw(51) << "*" <<
    std::endl << std::setfill(' ');
    std::cout << std::setw(25) << "| PARAMETERS:"       << std::setw(25) <<
    " "  << "|\n";
    std::cout << std::setw(25) << "| Threads:"           << std::setw(25) <<
    num_threads << "|\n";
    std::cout << std::setw(25) << "| Size X:"           << std::setw(25) <<
    sizeX_  << "|\n";
    std::cout << std::setw(25) << "| Size Y:"           << std::setw(25) <<
    sizeY_  << "|\n";
    std::cout << std::setw(25) << "| Fluid Tau:"        << std::setw(25) <<
    tauFluid_ << "|\n";
    std::cout << std::setw(25) << "| Iterations:"       << std::setw(25) <<
    iterations_ << "|\n";
    std::cout << std::setw(25) << "| Current iterations:" << std::setw(25) <<
    currentIteration_ << "|\n";
    std::cout << std::setw(25) << "| Report Steps:" << std::setw(25) <<
    reportSteps_ << "|\n";
    for (const auto& solver : cdeSolvers_) {
        std::cout << std::setw(15) << "| CDESolver:" << std::setw(10) <<
        "name:" << std::setw(25) << solver.first << "|\n";
        std::cout << std::setw(15) << "| " << std::setw(10) << "tau: " <<
        std::setw(25) << solver.second << "|\n";
    }
    std::cout << std::setfill('*') << std::setw(51) << "*" << std::endl;
    std::cout << std::setfill(' ');
}
}  // end namespace
