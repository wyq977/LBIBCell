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

#ifndef UTILIB_PROGRESSBAR_HPP_
#define UTILIB_PROGRESSBAR_HPP_

#include <UtilLib/include/Timer.hpp>
#include <iostream>

namespace UtilLib {
/**
 * @brief a class for reporting the current simulation status.
 * Usage:
 * construct an object of this class with the expected count
 * Use the increment operator on this object on each increased count
 * Then the current progress is printed to the output
 */
class ProgressBar {
 public:
    /**
     * Constructor
     * @param expected_count The expected count
     * @param updateInterval how often the information should be updated default this is 30 seconds
     * @param os The output stream
     */
    explicit ProgressBar(
            unsigned int expected_count,
            double updateInterval = 30,
            std::ostream& os = std::cout);
    /**
     * Display appropriate progress tic if needed.
     * @param increment
     * @post count()== original count() + increment
     * @return The increased integer
     */
    unsigned int operator+=(unsigned int increment);

    /**
     * @brief Prefix operator
     * @return the increased integer
     */
    unsigned int operator++();

    /**
     * @brief Postfix operator
     * @param i the increased integer
     * @return the increased integer
     */
    unsigned int operator++(int i);

 private:
    unsigned int count_;  ///< The current count
    unsigned int expectedCount_;  ///< The number of counts
    unsigned int nextTicCount_;  ///< When the next tic should be generated
    unsigned int updateCount_;  ///< When the percentage should be updated
    unsigned int tic_;  ///< The current tic
    /**
     * The stream where the progress Bar is printed to.
     */
    std::ostream& outputStream_;
    /**
     * @brief the interval in sec at which the percentage and runtime estimation is updated
     */
    const float updateInterval_;


    /**
     * use of floating point ensures that both large and small counts
     * work correctly.  static_cast<>() is also used several places
     * to suppress spurious compiler warnings.
     */
    void displayTic();

    /**
     * @brief updates the percentage.
     */
    void displayPercentage();

    /**
     * @brief The timers where wholeTime_ is used for the overall time
     */
    Timer wholeTime_;
};
} /* end namespace  */
#endif /* UTILIB_PROGRESSBAR_HPP_ */
