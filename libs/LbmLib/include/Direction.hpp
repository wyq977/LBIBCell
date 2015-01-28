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
#ifndef DIRECTION_HPP_
#define DIRECTION_HPP_

#include <UtilLib/include/Singleton.hpp>
#include <array>

namespace LbmLib {

/*!
 * \enum Direction
 * \brief Enum of D2Q9 directions.
 */
enum Direction {
    T = 0,  /**< resting direction = 0 */
    E = 1,  /**< east direction = 1 */
    N = 2,  /**< north direction = 2 */
    W = 3,  /**< west direction = 3 */
    S = 4,  /**< south direction = 4 */
    NE = 5, /**< northeast direction = 5 */
    NW = 6, /**< northwest direction = 6 */
    SW = 7, /**< southwest direction = 7 */
    SE = 8  /**< southeast direction = 8 */
};

/**
 * @brief getInverseDirection returns the inverse direction
 * @param d The direction
 * @return The inverse of d
 */
Direction getInverseDirection(const Direction& d);


/**
 * @brief The DirectionOperations_ class Provides methods to handle the Directions. Use the Function Directions() to access this class.
 */
class DirectionIterator {
 public:
    /**
     * constructor declared private
     */
    DirectionIterator() {}

    /**
     * non virtual destructor do not inherit from this class
     */
    ~DirectionIterator() {}

    /**
     * @brief begin To allow ranged based loop to iterate over this class
     * @return iterator to the begin
     */
    std::array<Direction, 9>::const_iterator begin() const;

    /**
     * @brief end To allow ranged based loop to iterate over this class
     * @return iterator to the end
     */
    std::array<Direction, 9>::const_iterator end() const;

 private:
    /**
     * @brief directions Internals storage for the directions
     */
    static std::array<Direction, 9> directions_;
};

/**
 * @brief The CDEDirectionsIteratorD2Q4 class Provides methods to handle the Directions. Use the Function Directions() to access this class.
 */
class CDEDirectionsIteratorD2Q4 {
 public:
    /**
     * constructor
     */
    CDEDirectionsIteratorD2Q4() {}

    /**
     * non virtual destructor do not inherit from this class
     */
    ~CDEDirectionsIteratorD2Q4() {}

    /**
     * @brief begin To allow ranged based loop to iterate over this class
     * @return iterator to the begin
     */
    std::array<Direction, 4>::const_iterator begin() const;

    /**
     * @brief end To allow ranged based loop to iterate over this class
     * @return iterator to the end
     */
    std::array<Direction, 4>::const_iterator end() const;

 private:
    /**
     * @brief directions Internals storage for the directions
     */
    static std::array<Direction, 4> directions_;
};


/**
 * @brief The CDEDirectionsIteratorD2Q5 class Provides methods to handle the Directions. Use the Function Directions() to access this class.
 */
class CDEDirectionsIteratorD2Q5 {
 public:
    /**
     * constructor
     */
    CDEDirectionsIteratorD2Q5() {}

    /**
     * non virtual destructor do not inherit from this class
     */
    ~CDEDirectionsIteratorD2Q5() {}

    /**
     * @brief begin To allow ranged based loop to iterate over this class
     * @return iterator to the begin
     */
    std::array<Direction, 5>::const_iterator begin() const;

    /**
     * @brief end To allow ranged based loop to iterate over this class
     * @return iterator to the end
     */
    std::array<Direction, 5>::const_iterator end() const;

 private:
    /**
     * @brief directions Internals storage for the directions
     */
    static std::array<Direction, 5> directions_;
};
}  // end namespace

#endif  /* DIRECTION_HPP_ */
