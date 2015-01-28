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
#ifndef UTILLIB_GEOMETRY_QUADRECTANGLE_HPP
#define UTILLIB_GEOMETRY_QUADRECTANGLE_HPP

namespace UtilLib {
namespace geometry {
/**
 * @brief a class representing a rectangle
 */
class Rectangle {
 public:
  /**
   * @brief Rectangle Constructs a rectangle
   * @param ymax The top border.
   * @param ymin The bottom border.
   * @param xmin The left border.
   * @param xmax The right border.
   */
  explicit Rectangle(double ymax, double ymin, double xmin, double xmax);

  /**
   * @brief Rectangle Constructs a rectangle around a point with lenght and hight size
   * @param x The x position
   * @param y The y position
   * @param size the lenght and height of the rectagle
   */
  explicit Rectangle(double x, double y, double size);

  /**
   * @brief within Is rect contained inside this rect
   * @param rect Another rectangele
   * @return True if they overlap
   */
  bool within(const Rectangle& rect) const;

  /**
   * @brief within Is rect contained inside this rect
   * @param n The north border of another rectangle
   * @param s The south border of another rectangle
   * @param w The west border of another rectangle
   * @param e The east border of another rectangle
   * @return True if they overlap
   */
  bool within(double n, double s, double w, double e) const;

  /**
   * @brief pointWithinBounds Checks if point is contained inside this rectangle
   * @param x The x pos of the point
   * @param y The y pos if the point
   * @return True of they overlap
   */
  bool pointWithinBounds(double x, double y) const;

  const double north;  ///< the north border
  const double south;  ///< the north border
  const double west;  ///< the north border
  const double east;  ///< the north border
};
}  // end namespace
}  // end namespace

#endif  // UTILLIB_GEOMETRY_QUADRECTANGLE_HPP
