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
#include <LbmLib/include/nodes/EulerianPoint.hpp>
#include <iostream>
#include <cmath>

namespace LbmLib {
namespace nodes {
EulerianPoint::EulerianPoint(
        int x,
        int y) : position_(x, y)
{}

int EulerianPoint::getXPos() const {
    return position_.x;
}

int EulerianPoint::getYPos() const {
    return position_.y;
}

Field<int> EulerianPoint::getPos() const {
    return position_;
}

std::ostream& operator<<(
        std::ostream& ostr,
        const EulerianPoint* pt) {
    ostr << "(" << pt->getXPos() << ":" << pt->getYPos() << ")";
    return ostr;
}
}
}  // end namespace
