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
#include <LbmLib/include/Direction.hpp>
#include <cassert>
#include <array>

namespace LbmLib {
Direction getInverseDirection(const Direction& d) {
    switch (d) {
      case T:
          return T;
          break;
      case E:
          return W;
          break;
      case N:
          return S;
          break;
      case W:
          return E;
          break;
      case S:
          return N;
          break;
      case NE:
          return SW;
          break;
      case NW:
          return SE;
          break;
      case SW:
          return NE;
          break;
      case SE:
          return NW;
          break;
      default:
          assert(
                false &&
                "you want to get a inverse direction of Direction that does not exist");
    }
}

std::array<Direction, 9>::const_iterator DirectionIterator::begin() const {
    return directions_.begin();
}

std::array<Direction, 9>::const_iterator DirectionIterator::end() const {
    return directions_.end();
}

std::array<Direction, 9> DirectionIterator::directions_ = {{T, E, N, W, S, NE, NW, SW, SE}};

std::array<Direction, 4>::const_iterator CDEDirectionsIteratorD2Q4::begin() const {
    return directions_.begin();
}

std::array<Direction, 4>::const_iterator CDEDirectionsIteratorD2Q4::end() const {
    return directions_.end();
}

std::array<Direction, 4> CDEDirectionsIteratorD2Q4::directions_ = {{E, N, W, S}};


std::array<Direction, 5>::const_iterator CDEDirectionsIteratorD2Q5::begin() const {
    return directions_.begin();
}

std::array<Direction, 5>::const_iterator CDEDirectionsIteratorD2Q5::end() const {
    return directions_.end();
}

std::array<Direction, 5> CDEDirectionsIteratorD2Q5::directions_ ={ {T, E, N, W, S}};
}  // end namespace
