/* Copyright (c) 2012 David Sichau <mail"at"sichau"dot"eu>

   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the
   "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish,
   distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to
   the following conditions:

   The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
   LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
   NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
   SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#ifndef UTILIB_GEOMETRY_QUADTREE_HPP
#define UTILIB_GEOMETRY_QUADTREE_HPP

#include <UtilLib/include/geometry/QuadTreeNode.hpp>
#include <UtilLib/include/geometry/Rectangle.hpp>
#include <memory>
#include <vector>
namespace UtilLib {

namespace geometry {
/**
 * @brief The class representing a quadtree
 * The template parameter Node needs to provide the methods getXPos() and getYPos() to access the postion of the node
 */
template<class Node>
class QuadTree {
 public:
    /**
     * @brief QuadTree A Quad tree on the area defined by the rectangle
     * @param rect The boarders of this quad tree
     * @param maxItems The maximum number of items stored per Node
     */
    explicit QuadTree(
            const Rectangle& rect,
            int maxItems = 1);
    /**
     * @brief ~QuadTree Destructor of this tree. At the moment no inheritance is wanted.
     */
    ~QuadTree();

    /**
     * @brief put Adds a point into the tree. This method runs in log(n)
     * @param pt The Point added
     * @return true if successfull
     */
    bool put(std::shared_ptr<Node> pt);

    /**
     * @brief get Get all points in the provided rectange. This method runs in log(n)
     * @param rect The rectange
     * @return All points inside rect
     */
    std::vector<std::shared_ptr<Node> > get(const Rectangle& rect);

    /**
     * @brief clear Delete all nodes inside this quadTree
     */
    void clear();

 private:
    /**
     * @brief top_ The top Node
     */
    QuadTreeNode<Node>* top_;
};



template<class Node>
QuadTree<Node>::QuadTree(
        const Rectangle& rect,
        int maxItems) {
    top_ = new QuadTreeNode<Node>(rect, maxItems);
}

template<class Node>
QuadTree<Node>::~QuadTree() {
    delete top_;
    top_ = nullptr;
}

template<class Node>
bool QuadTree<Node>::put(std::shared_ptr<Node> pt) {
    return top_->put(pt);
}

template<class Node>
std::vector<std::shared_ptr<Node> > QuadTree<Node>::get(
        const Rectangle& rect) {
    std::vector<std::shared_ptr<Node> > tempVec;
    return top_->get(rect, tempVec);
}

template<class Node>
void QuadTree<Node>::clear() {
    delete top_;
    top_ = nullptr;
}

}   // end namespace
}  // end namespace

#endif  // UTILIB_GEOMETRY_QUADTREE_HPP
