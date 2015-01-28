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
#ifndef FIELD_HPP
#define FIELD_HPP

namespace LbmLib {
/**
 * @brief The Field struct Storage of values in x and y direction
 */
template <typename T>
struct Field {
    /**
     * @brief Constructor
     */
    explicit Field() : x(0), y(0) {}

    /**
     * @brief Construtor
     *
     * @param xC the parameter for the x value
     * @param yC the parameter for the y value
     */
    explicit Field(
            T xC,
            T yC) : x(xC),
                    y(yC) {}

    /**
     * @brief x the value in x direction
     */
    T x;
    /**
     * @brief y the value in y direction
     */
    T y;

    /**
     * @brief The += operator
     *
     * @param rhs the righthand side field
     * @return this field + rhs field
     */
    Field<T>& operator+=(const Field<T>& rhs) {
        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    /**
     * @brief The -= operator
     *
     * @param rhs the righthand side field
     * @return this field - rhs field
     */
    Field<T>& operator-=(const Field<T>& rhs) {
        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }

    /**
     * @brief The *= operator
     *
     * @param rhs the righthand side field
     * @return this field * rhs field
     */
    Field<T>& operator*=(const Field<T>& rhs) {
        this->x *= rhs.x;
        this->y *= rhs.y;
        return *this;
    }

    /**
     * @brief The *= operator
     *
     * @param rhs a double value by which this field is multiplied
     * @return this field * rhs
     */
    Field<T>& operator*=(double rhs) {
        this->x *= rhs;
        this->y *= rhs;
        return *this;
    }

    /**
     * @brief The /= operator
     *
     * @param rhs the righthand side field
     * @return this field / rhs field
     */
    Field<T>& operator/=(double rhs) {
        this->x /= rhs;
        this->y /= rhs;
        return *this;
    }
};

/**
 * @brief the +operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs+rhs
 */
template <typename T>
inline Field<T> operator+(
        Field<T> lhs,
        const Field<T>& rhs) {
    lhs += rhs;
    return lhs;
}

/**
 * @brief the -operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs-rhs
 */
template <typename T>
inline Field<T> operator-(
        Field<T> lhs,
        const Field<T>& rhs) {
    lhs -= rhs;
    return lhs;
}

/**
 * @brief the *operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs*rhs
 */
template <typename T>
inline Field<T> operator*(
        Field<T> lhs,
        const Field<T>& rhs) {
    lhs *= rhs;
    return lhs;
}

/**
 * @brief the *operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs*rhs
 */
template <typename T>
inline Field<T> operator*(
        Field<T> lhs,
        double rhs) {
    lhs *= rhs;
    return lhs;
}

/**
 * @brief the *operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs*rhs
 */
template <typename T>
inline Field<T> operator*(
        double lhs,
        Field<T> rhs) {
    rhs *= lhs;
    return rhs;
}

/**
 * @brief the /operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs/rhs
 */
template <typename T>
inline Field<T> operator/(
        Field<T> lhs,
        double rhs) {
    lhs /= rhs;
    return lhs;
}

/**
 * @brief the /operator
 *
 * @tparam T the type of the Field
 * @param lhs
 * @param rhs
 *
 * @return lhs/rhs
 */
template <typename T>
inline Field<T> operator/(
        double lhs,
        Field<T> rhs) {
    rhs /= lhs;
    return rhs;
}
}  // end namespace

#endif  // FIELD_HPP
