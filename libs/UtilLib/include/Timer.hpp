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

#ifndef UTILIB_TIMER_HPP_
#define UTILIB_TIMER_HPP_

#pragma once
#include <sys/time.h>

namespace UtilLib {
/**
 * @brief a class for timing
 */
class Timer {
 public:
    /**
     * @brief starts the timer
     */
    void start() {
        gettimeofday(&t_start,  &t_zone);
    }

    /**
     * @brief stops the timer and returns the time in sec
     *
     * @return the time since the timer has started
     */
    inline double stop() {
        gettimeofday(&t_end,  &t_zone);
        return (t_end.tv_usec  -
                t_start.tv_usec) * 1e-6  + (t_end.tv_sec  - t_start.tv_sec);
    }

 private:
    /**
     * @brief storage for the start and end time
     */
    struct timeval t_start, t_end;
    /**
     * @brief stores the timezone
     */
    struct timezone t_zone;
};
} /* end namespace  */
#endif /* UTILIB_TIMER_HPP_ */
