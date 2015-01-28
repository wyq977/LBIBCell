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
#ifndef EULERIANPOINT_HPP
#define EULERIANPOINT_HPP

#include <LbmLib/include/Field.hpp>
#include <string>
#include <iostream>
#include <cmath>
namespace LbmLib {
namespace nodes {
/**
 * @brief The EulerianPoint class The base class for all points with an integer position and no speed.
 */
class EulerianPoint {
 public:
    /**
     * @brief EulerianPoint Constructs a new EulerianPoint
     * @param x The x position
     * @param y The y position
     */
    explicit EulerianPoint(
            int x,
            int y);

    /**
     * @brief ~Point vitual Destructor
     */
    virtual ~EulerianPoint() {}

    /**
     * @brief getXPos Getter for the X position
     * @return the x pos
     */
    int getXPos() const;

    /**
     * @brief getYPos Getter for the Y position
     * @return the y pos
     */
    int getYPos() const;

    /**
     * @brief getPos Getter for the pos
     * @return The position
     */
    Field<int> getPos() const;

    /**
     * @brief getType The type of a node class
     * @return Returns the class Name of the point
     */
    virtual std::string getType() const = 0;

 private:
    /**
     * @brief position_ Stores the position of a point.
     */
    const Field<int> position_;
};


/**
 * @brief getSquaredDistance Calculates the squared distance between pt1 and pt2. Use this if only comparision is needed as it avoids a root calculation
 * @param pt1 A Point which provides getXPos() and getYPos()
 * @param pt2 A Point which provides getXPos() and getYPos()
 * @return The squared distance between pt1 and pt2
 */
template <typename Point1, typename Point2>
double getSquaredDistance(
        const Point1& pt1,
        const Point2& pt2) {
    return (
                (pt1.getXPos() - pt2.getXPos()) * (pt1.getXPos() - pt2.getXPos()) +
                (pt1.getYPos() - pt2.getYPos()) * (pt1.getYPos() - pt2.getYPos())
            );
}

/**
 * @brief getDistance Calculates the distance between pt1 and pt2
 * @param pt1 A Point which provides getXPos() and getYPos()
 * @param pt2 A Point which provides getXPos() and getYPos()
 * @return The distance between pt1 and pt2
 */
template <typename Point1, typename Point2>
double getDistance(
        const Point1& pt1,
        const Point2& pt2) {
    return std::sqrt(getSquaredDistance(pt1, pt2));
}

/**
 * @brief getDistanceField Calculates the distance between pt1 and pt2
 * @param pt1 A Point which provides getXPos() and getYPos()
 * @param pt2 A Point which provides getXPos() and getYPos()
 * @return The distance Field between pt1 and pt2
 */
template <typename Point1, typename Point2>
Field<double> getDistanceField(
        const Point1& pt1,
        const Point2& pt2) {
    return Field<double>(pt2.getXPos() - pt1.getXPos(),
                         pt2.getYPos() - pt1.getYPos());
}

/**
 * @brief overload for EulerianPoint for ostream
 * @param ostr the ostream
 * @param pt a point
 *
 * @return the ostream
 */
std::ostream& operator<<(
        std::ostream& ostr,
        const EulerianPoint* pt);

/**
 * @brief isPointInTriangle returns true if pt4 is contained in triangle defined by pt1,pt2,pt3
 * @param pt1 Triangle Edge
 * @param pt2 Triangle Edge
 * @param pt3 Triangle Edge
 * @param pt4 Point which is checked if it is in Triangle
 * @return true if pt4 is contained.
 */
template <typename Pt1, typename Pt2, typename Pt3, typename Pt4>
bool isPointInTriangle(
        const Pt1& pt1,
        const Pt2& pt2,
        const Pt3& pt3,
        const Pt4& pt4) {
    // Barycentric coordinate system
    double lambda1 =
        ((pt2.getYPos() - pt3.getYPos()) * (pt4.getXPos() - pt3.getXPos()) +
            (pt3.getXPos() -
                       pt2.getXPos()) * (pt4.getYPos() - pt3.getYPos())) /
        ((pt2.getYPos() - pt3.getYPos()) * (pt1.getXPos() - pt3.getXPos()) +
         (pt3.getXPos() - pt2.getXPos()) * (pt1.getYPos() - pt3.getYPos()));
    double lambda2 =
        ((pt3.getYPos() - pt1.getYPos()) * (pt4.getXPos() - pt3.getXPos()) +
            (pt1.getXPos() -
                       pt3.getXPos()) * (pt4.getYPos() - pt3.getYPos())) /
        ((pt2.getYPos() - pt3.getYPos()) * (pt1.getXPos() - pt3.getXPos()) +
         (pt3.getXPos() - pt2.getXPos()) * (pt1.getYPos() - pt3.getYPos()));
    double lambda3 = 1 - lambda1 - lambda2;

    if ((lambda1 <= 0) || (lambda2 <= 0) || (lambda3 <= 0) || (lambda1 >= 1) ||
        (lambda2 >= 1) || (lambda3 >= 1) ) {
        return false;
    }
    return true;
}
}   // end namespace
}  // end namespace
#endif  // EULERIANPOINT_HPP
