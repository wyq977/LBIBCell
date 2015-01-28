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
#ifndef ABSTRACTREPORTFUNCTOR_HPP
#define ABSTRACTREPORTFUNCTOR_HPP

#include <string>

namespace LbmLib {
namespace reportHandler {
/**
 * @brief the abstract reporter functor inherit from this for your reporters
 */
class AbstractReportFunctor {
 public:
    /**
     * @brief ReportFunctor The constructor of a reporter functor which takes a name
     * @param filename The filename which is used by this reporter
     */
    explicit AbstractReportFunctor(const std::string& filename) : filename_(
                                                                          filename)
    {}

    /**
     * @brief ~ReportFunctor virtual destructor
     */
    virtual ~AbstractReportFunctor() {}

    /**
     * @brief operator() This method needs to be overriden by base classes. This should write the report.
     * @param time The simulation time at which this method is called
     */
    virtual void operator()(unsigned int time) const = 0;

 protected:
    /**
     * @brief filename_ Stores the filename of this functor
     */
    const std::string filename_;
};
}
}  // end namespace

#endif  // ABSTRACTREPORTFUNCTOR_HPP
