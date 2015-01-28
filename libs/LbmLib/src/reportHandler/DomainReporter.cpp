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
#include <LbmLib/include/reportHandler/DomainReporter.hpp>
#include <UtilLib/include/Exception.hpp>
#include <LbmLib/include/nodes/PhysicalNode.hpp>
#include <LbmLib/include/solver/CDESolver/CDEAbstractSolver.hpp>
#include <LbmLib/include/solver/FluidSolver/FluidSolver.hpp>
#include <sstream>
#include <map>
#include <fstream>
#include <iomanip>

namespace LbmLib {
namespace reportHandler {
void DomainReporter::operator()(unsigned int time) const {
    std::stringstream filename;
    filename << filename_ << "_" << time << ".txt";
    std::ofstream oStream(filename.str().c_str());
    if (!oStream.is_open()) {
        UtilLib::Exception("The output file could not be opened.");
    }

    std::map<unsigned int, double> sums;
    for (auto i : physicalNodes_) {
        for (auto pt : i) {
            oStream << pt->getXPos() << "\t" << pt->getYPos() << "\t" <<
                    pt->getDomainIdentifier() << "\n";
        }
    }
    oStream.close();
}
}
}  // end namespace
