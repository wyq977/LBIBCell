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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#include <UtilLib/include/geometry/Rectangle.hpp>
namespace UtilLib {
namespace geometry {
Rectangle::Rectangle(
        double n,
        double s,
        double w,
        double e) : north(n),
                    south(s),
                    west(w),
                    east(e)
{}

Rectangle::Rectangle(
        double x,
        double y,
        double size) : north(y + size / 2),
                       south(y - size / 2),
                       west(x - size / 2),
                       east(x + size / 2) {}

bool Rectangle::within(const Rectangle& rect) const {
    return within(rect.north, rect.south, rect.west, rect.east);
}

bool Rectangle::within(
        double n,
        double s,
        double w,
        double e) const {
    if (s > north || s < south)
        return false;
    if (n < south || n > north)
        return false;
    if (w > east || w < west)
        return false;
    if (e < west || e > east)
        return false;
    return true;
}

bool Rectangle::pointWithinBounds(
        double x,
        double y) const {
    if ((x >= west) && (x < east) && (y <= north) && (y > south)) {
        return true;
    } else {
        return false;
    }
}
}   // end namespace
}  // end namespace
