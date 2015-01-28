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
#include <LbmLib/include/reportHandler/ReportHandler.hpp>
#include <LbmLib/include/reportHandler/AbstractReportFunctor.hpp>

namespace LbmLib {
namespace reportHandler {
/**
 * @brief registerReporter Register a report Functor. This method takes ownership of the unique_ptr passed
 * @param reporter The reporter added to the report Handler
 */
void ReportHandler::registerReporter(
        std::unique_ptr<AbstractReportFunctor> reporter) {
    reporters_.push_back(std::move(reporter));
}

/**
 * @brief writeReport Writes the report if the time step fits to the reportStep
 * @param time The current time
 */
void ReportHandler::writeReport(unsigned int time) const {
    if (time % reportStep_ == 0) {
        for (const auto& i : this->reporters_) {
            (* i)(time);
        }
    }
}

void ReportHandler::writeCrashReport(unsigned int time) const
{
    for (const auto& i : this->reporters_) {
        (* i)(time);
    }
}
}
}  // end namespace
