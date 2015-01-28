/* Copyright (c) 2013 David Sichau <mail"at"sichau"dot"eu>
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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#include <UtilLib/config.hpp>
#include <UtilLib/include/MPIProxy.hpp>
#include <UtilLib/include/Log.hpp>
#include <UtilLib/include/Exception.hpp>
#include <iomanip>
#include <string>

namespace UtilLib {
namespace {
/**
 * helper function to convert debug levels to stings
 * @param level the debug level
 * @return the string of the debug level
 */
std::string logLevelToString(const LogLevel& level) {
    switch (level) {
      case logERROR:
          return std::string("Error");
          break;
      case logWARNING:
          return std::string("Warning");
          break;
      case logINFO:
          return std::string("Info");
          break;
      case logDEBUG:
          return std::string("Debug");
          break;
      case logDEBUG1:
          return std::string("Debug1");
          break;
      case logDEBUG2:
          return std::string("Debug2");
          break;
      case logDEBUG3:
          return std::string("Debug3");
          break;
      case logDEBUG4:
          return std::string("Debug4");
          break;
      default:
          break;
    }
    return std::string("");
}

/**
 * @brief a delete which does not delete anything
 */
struct null_deleter {
    /**
     * @brief functor to avoid deletion of an object that couldnot be deleted nothing at delete
     */
    void operator()(void const*) const {}
};
}

/*
 * The default log level is set to the compile flag DEBUGLEVEL.
 */
LogLevel Log::reportingLevel_ = DEBUGLEVEL;

/// @cond
// Default the log is printed to std::cerr. To avoid the deletion of std::cerr a null_deleter is provided.
std::shared_ptr<std::ostream> Log::pStream_(&std::cerr, null_deleter());
/// @endcond
std::shared_ptr<std::ostream> Log::getStream() {
    return pStream_;
}

void Log::setStream(std::shared_ptr<std::ostream> pStream) {
    if (!pStream) {
        throw Exception(
                "The stream is not available. You can only set the stream to an existing one.");
    }
    pStream_ = pStream;
}

void Log::writeOutput(const std::string& msg) {
    std::shared_ptr<std::ostream> pStream = getStream();
    (*pStream) << msg;
    pStream->flush();
}

std::ostringstream& Log::writeReport(LogLevel level) {
    // generate time in the format Date HH::MM::SS
    time_t rawtime;
    time(&rawtime);
    struct tm tempTm1;
    struct tm* tempTm2;
    tempTm2 = localtime_r(&rawtime, &tempTm1);
    char outstr[200];
    strftime(outstr, sizeof(outstr), "%x% %H:%M:%S", tempTm2);

    buffer_ << "- " << outstr;
    buffer_ << " Proc " << MPIProxy().getRank() << " of "
            << MPIProxy().getSize();
    buffer_ << std::setw(10) << logLevelToString(level) << ":\t";
    return buffer_;
}

void Log::setReportingLevel(LogLevel level) {
    LOG(logINFO) << "Report Level changed from "
                 << logLevelToString(reportingLevel_) << " to "
                 << logLevelToString(level);
    reportingLevel_ = level;
}

LogLevel Log::getReportingLevel() {
    return reportingLevel_;
}

Log::~Log() {
    buffer_ << std::endl;
    Log::writeOutput(buffer_.str());
}
}
