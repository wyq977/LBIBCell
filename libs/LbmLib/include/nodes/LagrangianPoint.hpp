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
#ifndef POINT_HPP
#define POINT_HPP

#include <LbmLib/include/Field.hpp>
#include <string>
#include <iostream>
namespace LbmLib {
namespace nodes {
class EulerianPoint;
/**
 * @brief The LagrangianPoint class A class for storing a point's coordinates and its velocity
 * This class is the base class for all points which have a velocity and can be at any position.
 */
class LagrangianPoint {
 public:
    /**
     * @brief LagrangianPoint Constructs a new point
     * @param x The x position
     * @param y The y position
     */
    explicit LagrangianPoint(
            double x,
            double y);

    /**
     * @brief ~LagrangianPoint vitual Destructor
     */
    virtual ~LagrangianPoint() {}

    /**
     * @brief getXVelocity Getter for x-velocity component
     * @return the x-velocity component
     */
    double getXVelocity() const;

    /**
     * @brief getYVelocity Getter for the y-velocity component
     * @return the x-velocity component
     */
    double getYVelocity() const;

    /**
     * @brief getVelocity Getter for the velocity
     * @return The velocity
     */
    Field<double> getVelocity() const;

    /**
     * @brief getXPos Getter for the X position
     * @return the x pos
     */
    double getXPos() const;

    /**
     * @brief getYPos Getter for the Y position
     * @return the y pos
     */
    double getYPos() const;

    /**
     * @brief getPos Getter for the pos
     * @return The position
     */
    Field<double> getPos() const;

    /**
     * @brief setPos Set the position new
     * override this method to change default behaviour
     * @param x The new x position
     * @param y The new y position
     */
    virtual void setPos(
            double x,
            double y);

    /**
     * @brief setXPos Set a new x position.
     * @param x The new x position.
     */
    void setXPos(
            double x);

    /**
     * @brief setYPos Set a new y position.
     * @param y The new y position
     */
    void setYPos(
            double y);

    /**
     * @brief setVelocity Set the velocity
     * @param velocity The new velocity
     */
    virtual void setVelocity(Field<double> velocity);

    /**
     * @brief getType The type of a node class
     * @return Returns the class Name of the point
     */
    virtual std::string getType() const;

 private:
    /**
     * @brief position_ Stores the position of a point
     */
    Field<double> position_;

    /**
     * @brief velocity_ Stores the velocity of the point
     */
    Field<double> velocity_;
};
/**
 * @brief overload for LagrangianPoint for ostream
 * @param ostr the ostream
 * @param pt a point
 *
 * @return the ostream
 */
std::ostream& operator<<(
        std::ostream& ostr,
        const LagrangianPoint* pt);
}   // end namespace
}  // end namespace

#endif  // POINT_HPP
