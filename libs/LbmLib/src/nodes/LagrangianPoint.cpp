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
#include <LbmLib/include/nodes/LagrangianPoint.hpp>
#include <LbmLib/include/nodes/EulerianPoint.hpp>
#include <LbmLib/include/Constants.hpp>
#include <UtilLib/include/Log.hpp>
#include <UtilLib/include/Exception.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <cassert>
#include <iomanip>

namespace LbmLib {
namespace nodes {
LagrangianPoint::LagrangianPoint(
        double x,
        double y) : position_(x, y)
{
    assert(std::isfinite(this->position_.x));
    assert(std::isfinite(this->position_.y));
}

double LagrangianPoint::getXVelocity() const {
    return this->velocity_.x;
}

double LagrangianPoint::getYVelocity() const {
    return this->velocity_.y;
}

Field<double> LagrangianPoint::getVelocity() const {
    assert(std::isfinite(this->velocity_.x));
    assert(std::isfinite(this->velocity_.y));
    return this->velocity_;
}

double LagrangianPoint::getXPos() const {
    return this->position_.x;
}

double LagrangianPoint::getYPos() const {
    return this->position_.y;
}

Field<double> LagrangianPoint::getPos() const {
    return this->position_;
}

void LagrangianPoint::setPos(
        double x,
        double y) {
    this->setXPos(x);
    this->setYPos(y);
}

void LagrangianPoint::setXPos(double x)
{
    this->position_.x = x;
    assert(std::isfinite(this->position_.x));
}

void LagrangianPoint::setYPos(double y)
{
    this->position_.y = y;
    assert(std::isfinite(this->position_.y));
}

std::string LagrangianPoint::getType() const {
    return "LagrangianPoint";
}

void LagrangianPoint::setVelocity(Field<double> velocity) {
    this->velocity_ = velocity;
    assert(std::isfinite(this->velocity_.x));
    assert(std::isfinite(this->velocity_.y));
    if (((this->velocity_.x * this->velocity_.x) + (this->velocity_.y * this->velocity_.y)) > CS) {
        std::stringstream message;
        message <<std::setprecision(12);
        message << "The magnitude of velocity is too high: vx=" << this->velocity_.x << ", vy=" << this->velocity_.y;
        LOG(UtilLib::logINFO) << message;
        lbm_fail(message.str().c_str());
    }
}

std::ostream& operator<<(
        std::ostream& ostr,
        const LagrangianPoint* pt) {
    ostr << "(" << pt->getXPos() << ":" << pt->getYPos() << ")";
    return ostr;
}
}
}  // end namespace
