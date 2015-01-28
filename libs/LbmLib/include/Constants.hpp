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
#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

#include <cmath>

namespace LbmLib {
/**
 * @brief CS The speed of sound
 */
constexpr double CS = 1 / std::sqrt(3);
/**
 * @brief EPSILON The delta used in double comparisons
 */
const double EPSILON = 10E-10; //10
/**
 * @brief PERTURBATION The perturbation used to avoid clashes
 */
const double PERTURBATION = 10E-7; //8

/**
 * @brief PI Pi
 */
constexpr double PI = std::atan(1) * 4;

/**
 * @brief Maximal connection length. If exceeded, boundary remeshing is required.
 */
const double MAXLENGTH = 0.5;

/**
 * @brief Minimum number of elements in boost::geometry::index::rstar.
 */
const std::size_t BGIRSTARMINELEMENTS = 4;

/**
 * @brief Maximal number of elements in boost::geometry::index::rstar.
 */
const std::size_t BGIRSTARMAXELEMENTS = 16;

/**
 * @brief MAXBINSIZE The maximal bin size for the FastNeighborList.
 */
constexpr double MAXBINSIZE = 1.0;

}  // end namespace

#endif  // CONSTANTS_HPP
