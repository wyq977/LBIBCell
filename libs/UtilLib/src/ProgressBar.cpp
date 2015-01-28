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
#include <UtilLib/include/ProgressBar.hpp>
#include <UtilLib/include/TerminalColors.hpp>
#include <iomanip>
#include <vector>

namespace UtilLib {
ProgressBar::ProgressBar(
        unsigned int expectedCount,
        double updateInterval,
        std::ostream& os)
    : count_(0),
      expectedCount_(expectedCount),
      nextTicCount_(0),
      updateCount_(0),
      tic_(0),
      outputStream_(os),
      updateInterval_(updateInterval) {
    outputStream_ << "progress: " << std::setw(4) << std::setprecision(3) <<
    static_cast<float>(tic_) << "%   estimated time remaining:" << std::setw(
            6) << std::setprecision(3) << "inf sec\n"
    << "0%   10   20   30   40   50   60   70   80   90   100%\n"
    << "|----|----|----|----|----|----|----|----|----|----|"
    << std::endl;
    wholeTime_.start();
}

unsigned int ProgressBar::operator+=(unsigned int increment) {
    count_ += increment;
    if (count_ >= nextTicCount_) {
        displayPercentage();
        displayTic();
    } else if (count_ > updateCount_) {
        displayPercentage();
    }
    return count_;
}

unsigned int ProgressBar::operator++() {
    return operator+=(1);
}

unsigned int ProgressBar::operator++(int i) {
    return operator+=(1);
}

void ProgressBar::displayTic() {
    unsigned int tics_needed =
        static_cast<unsigned int>((static_cast<double>(count_) /
                                   expectedCount_) * 50.0);
    do {
        outputStream_ << '*' << std::flush;
    } while (++tic_ < tics_needed);

    nextTicCount_ = static_cast<unsigned int>((tic_ / 50.0) * expectedCount_);
    // if the end is reached make sure that the output is correct
    if (count_ == expectedCount_) {
        outputStream_ << STORE_CURSOR << GO_LINE_UP << GO_LINE_UP << GO_LINE_UP
                      << GO_LINE_BEGIN << "progress: " << std::setw(4)
                      << std::setprecision(3) << static_cast<float>(100)
                      << "%   estimated time remaining:" << std::setw(6)
                      << std::setprecision(3) << 0.0 << " sec" << DEL_END
                      << RESET_CURSOR;
        if (tic_ < 51)
            outputStream_ << '*';

        outputStream_ << "\noverall runtime: " << std::setw(9) << BOLDWHITE
                      << std::setprecision(6) << wholeTime_.stop() << " sec"
                      << RESET << std::endl;
    }
}

void ProgressBar::displayPercentage() {
    const float percentage = static_cast<float>(count_) / (expectedCount_) *
        100;
    const float dt = wholeTime_.stop();
    const float remainingTime = 100 * dt / percentage - dt;
    updateCount_ = count_ + count_ / dt * updateInterval_;
    /*
     * I use the following terminal control sequences:
     * \x1b[s stores the current cursor position
     * \x1b[A goes one line up
     * \r goes to the beginning of the line
     * \x1b[K deletes form the cursor position to the end
     * \x1b[u restores the cursor position
     */
    outputStream_ << STORE_CURSOR << GO_LINE_UP << GO_LINE_UP << GO_LINE_UP
                  << GO_LINE_BEGIN << "progress: " << std::setw(4)
                  << BOLDWHITE << std::setprecision(3) << percentage << "%   "
                  << RESET << "estimated time remaining: " << std::setw(6)
                  << BOLDWHITE << std::setprecision(3) << remainingTime << RESET
                  << " sec" << DEL_END << RESET_CURSOR << std::flush;
}
} /* end namespace  */
