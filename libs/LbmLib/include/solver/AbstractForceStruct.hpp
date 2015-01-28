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
#ifndef ABSTRACTFORCESTRUCT_HPP
#define ABSTRACTFORCESTRUCT_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace LbmLib {
namespace nodes {
class GeometryNode;
}

namespace solver {
/**
 * @brief the base class for all forces
 */
struct AbstractForceStruct {
    /**
     * @brief calculates the force on the geometry nodes
     * @param nodes the nodes where the force is applied
     */
    virtual void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes) = 0;

    /**
     * @brief Destructor
     */
    virtual ~AbstractForceStruct() {}

    /**
     * @brief writes the force struct to the ostream
     * @param stream the output stream
     */
    virtual void writeForceStruct(std::ofstream* const stream) const = 0;

    /**
     * @brief Returns the force type.
     * @return The force type.
     */
    virtual unsigned int getType(void) const = 0;

    /**
     * @brief getPartnerGeometryNode
     * @return Shall return 0 if there is no partner node, or its nodeID otherwise.
     */
    virtual unsigned int getPartnerGeometryNode(void) const = 0;
};

}
}

#endif // ABSTRACTFORCESTRUCT_HPP
