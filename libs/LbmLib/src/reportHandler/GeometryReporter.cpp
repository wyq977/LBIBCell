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
#include "LbmLib/include/reportHandler/GeometryReporter.hpp"
#include <UtilLib/include/Exception.hpp>
#include <LbmLib/include/solver/FluidSolver/FluidSolver.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <sstream>
#include <fstream>
#include <iomanip>

namespace LbmLib {
namespace reportHandler {



void GeometryReporter::operator()(unsigned int time) const {
    typedef std::map<unsigned int, std::shared_ptr<nodes::GeometryNode> >::const_iterator MapIterator;

    std::stringstream filename;
    filename << filename_ << "_" << time << ".txt";
    std::ofstream oStream(filename.str().c_str());
    if (!oStream.is_open()) {
        UtilLib::Exception("The output file could not be opened.");
    }

    // write:
    for (MapIterator it = this->GeometryNodes_.begin();
         it != this->GeometryNodes_.end(); it++)
    {
        oStream << std::setprecision(12) <<
                   it->second->getId() << "\t" <<
                   it->second->getXPos() << "\t" <<
                   it->second->getYPos() << "\t" <<
                   it->second->getXVelocity() << "\t" <<
                   it->second->getYVelocity() << "\n";
    }

    oStream.close();
}
}
}  // end namespace
