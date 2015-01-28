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

#ifndef UTILIB_LOG_HPP_
#define UTILIB_LOG_HPP_

#include <time.h>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace UtilLib {
/*! \page logging The Log utilities provided by miind
 * This page contains the following sections:
 * <ol>
 * <li>\ref logging_introduction</li>
 * <li>\ref advanced_use</li>
 * <li>\ref details_macro</li>
 * <li>\ref provided_debug_levels</li>
 * </ol>
 * \section logging_introduction Introduction
 * To log a message in miind use the following macro:
 * @code{.cpp}
 * LOG(UtilLib::logWARNING)<<"blub: "<<42;
 * @endcode
 * This would then log a message of the level logWARNING if the current reporting level
 * is higher that logWARNING. Otherwise the logging would be ignored. As this check is
 * done at compile time you pay only for log messages if they are actually printed.
 *
 * \section advanced_use Advanced use of logging
 * The default logging level is defined by the flag DEBUGLEVEL then everything is printed to the log.
 * To change the reporting level of the log class the following code is needed:
 *
 * @code{.cpp}
 *      UtilLib::Log::setReportingLevel(UtilLib::logWARNING);
 * @endcode
 *
 * This code would set the reporting level to \c logWARNING
 *
 * In the default version log messages are printed to std::cerr. To print the log messages
 * into a log file the following code is needed:
 *
 * @code{.cpp}
 *      std::shared_ptr<std::ostream> pStream( new std::ofstream("MYLOGFILENAME"));
 *	if (!pStream){
 *         throw UtilLib::Exception("cannot open log file.");
 *	}
 *	UtilLib::Log::setStream(pStream);
 * @endcode
 *
 * This code would redirect the log messages to the file with the name MYLOGFILENAME.
 *
 * \section details_macro The Log Macro
 *
 * The macro allows easier generation of log messages.
 * It also improves the efficiency significantly as the checks are conducted at compile time.
 * @attention do not pass functions to this macro. This is due to the problematic macro expansion.
 * For example this:
 *
 * @code{.cpp}
 * LOG(logERROR)<<getNumber();
 * @endcode
 *
 * is forbidden. Please use then instead a temporary variable:
 *
 * @code{.cpp}
 * int number = getNumber()
 * LOG(logERROR)<<number;
 * @endcode
 *
 * or alternatively write it the following way:
 *
 * @code{.cpp}
 * if(level > UtilLib::Log::getReportingLevel() || !UtilLib::Log::getStream()){
 *     ;
 * }else{
 *     UtilLib::Log().writeReport(level)<<getNumber;
 * }
 * @endcode
 * However try to use the macro with temporary variables.
 *
 * \section provided_debug_levels Provided Debug levels
 *
 * <dl>
 * <dt>logERROR</dt>
 * <dd>Only use this for real error messages, as these are always logged.</dd>
 * <dt>logWARNING</dt>
 * <dd>use this for warnings messages.</dd>
 * <dt>logINFO</dt>
 * <dd>Use this for information messages</dd>
 * <dt>logDEBUG</dt>
 * <dd>Use this for very important debug messages</dd>
 * <dt>logDEBUG1</dt>
 * <dd>Use this for important debug messages</dd>
 * <dt>logDEBUG2</dt>
 * <dd>Use this for not so important debug messages</dd>
 * <dt>logDEBUG3</dt>
 * <dd>Use this for not important debug messages</dd>
 * <dt>logDEBUG4</dt>
 * <dd>Use this for every debug detail messages</dd>
 * </dl>
 *
 */

/**
 * The log levels for more details see \ref provided_debug_levels
 */
enum LogLevel {
    logERROR,  // !< logERROR
    logWARNING,  // !< logWARNING
    logINFO,   // !< logINFO
    logDEBUG,  // !< logDEBUG
    logDEBUG1,  // !< logDEBUG1
    logDEBUG2,  // !< logDEBUG2
    logDEBUG3,  // !< logDEBUG3
    logDEBUG4   // !< logDEBUG4
};

/**
 * @brief class for logging reports. The usage of this log class is described on page \ref logging
 */
class Log {
 public:
    /**
     * default constructor
     */
    Log() = default;
    /**
     * copy constructor deleted
     */
    Log(const Log&) = delete;
    /**
     * copy operator deleted
     * @return the copy of log
     */
    Log& operator =(const Log&) = delete;

    /**
     * destructor which writes the message to the stream
     */
    virtual ~Log();

    /**
     * takes the log message and stores it in the buffer
     * @param level The level of the log message
     * @return  a ostringstream
     */
    std::ostringstream& writeReport(LogLevel level = logINFO);

    /**
     * The Stream it writes to the standard is std::cerr
     * @return The Stream it writes to
     */
    static std::shared_ptr<std::ostream> getStream();

    /**
     * Setter for the stream
     * @param pStream The Stream the log class should print to.
     */
    static void setStream(std::shared_ptr<std::ostream> pStream);

    /**
     * getter for the current report level
     * @return the current report level
     */
    static LogLevel getReportingLevel();

    /**
     * setter for the report level
     * @param level The new report level
     */
    static void setReportingLevel(LogLevel level);

 private:
    /**
     * writes the output to the stream
     * @param msg The message written to the stream
     */
    static void writeOutput(const std::string& msg);

    /**
     * The current reporting level of the Log, all messages with a level below this level
     * are printed the rest is ignored
     */
    static LogLevel reportingLevel_;

    /**
     * pointer to the stream
     */
    static std::shared_ptr<std::ostream> pStream_;
    /**
     * The buffer for the log message
     */
    std::ostringstream buffer_;
};
}  /* end namespace */

/**
 * macro to allow easier generation of log messages.
 * this will improve the efficiency significantly as the checks are conducted at compile time.
 * see also \ref details_macro for more details
 */
#define LOG(level) \
    if (level > UtilLib::Log::getReportingLevel() || !UtilLib::Log::getStream()) ; \
    else UtilLib::Log().writeReport(level)



#endif /* UTILIB_LOG_HPP_ */
