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
#ifndef UTILLIB_GEOMETRY_QUADTREENODE_HPP
#define UTILLIB_GEOMETRY_QUADTREENODE_HPP

#include <UtilLib/include/geometry/Rectangle.hpp>
#include <UtilLib/include/Log.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <vector>


namespace UtilLib {
namespace geometry {
/**
 * @brief class representing a node of a quadtree
 * The template parameter Node needs to provide the methods getXPos() and getYPos() to access the postion of the node
 */
template<class Node>
class QuadTreeNode {
 public:
    /**
     * @brief QuadTreeNode Constructs a new QuadTreeNode
     * @param rect The boarders of this node
     * @param maximumItems The number of items stored in one leaf
     */
    explicit QuadTreeNode(
            const Rectangle& rect,
            int maximumItems);

    /**
     * Destructor
     **/
    ~QuadTreeNode();

    /**
     * @brief put Inserts a point into the Node
     * @param pt A Pointer to a point stored in the tree. However the quadTree does not take over the ownership
     * @return True if the method was successful
     */
    bool put(std::shared_ptr<Node> pt);

    /**
     * @brief get Get all Points in the defined rectangle
     * @param rect The rectange
     * @param vector A vector of all Points found so far
     * @return A vector of Pointer to the nodes in the defined area
     */
    std::vector<std::shared_ptr<Node> >& get(
            const Rectangle& rect,
            std::vector<std::shared_ptr<Node> >& vector);

 private:
    /**
     * @brief split Does the split operation on this leaf
     */
    void split();

    /**
     * @brief getChild Get the children which corrrespond to the defined Position
     * @param x The x position
     * @param y The y position
     * @return The Children at that position
     */
    QuadTreeNode* getChild(
            double x,
            double y);

    /**
     * @brief hasChildren Query to find out if the node has children
     * @return True if the node has children
     */
    bool hasChildren();

    /**
     * @brief The Regions enum Defines the four possible areas of a quadNode
     */
    enum Regions {NORTHWEST = 0, NORTHEAST = 1, SOUTHEAST = 2, SOUTHWEST = 3};

    /**
     * @brief items_ A vector of the actual stored items
     */
    std::vector<std::shared_ptr<Node> > items_;

    /**
     * @brief children_ A Pointer to the children of this node
     */
    std::array<QuadTreeNode*, 4>* children_;

    /**
     * @brief maxItems_ The max number of items stored in one Node
     */
    unsigned int maxItems_;
    /**
     * @brief bounds_ The bounds of this Node
     */
    Rectangle bounds_;

    /**
     * @brief minSize_ Defines the minimum size of a node to avoid endless spliting if they are to close
     */
    const double minSize_;
};

template<class Node>
QuadTreeNode<Node>::QuadTreeNode(
        const Rectangle& rect,
        int maximumItems) : children_(nullptr),
                            maxItems_(maximumItems),
                            bounds_(rect),
                            minSize_(1E-4) {
    LOG(UtilLib::logDEBUG2) << "new node generated with boarder n: " <<
    bounds_.north << " s: " << bounds_.south << " w: " <<
    bounds_.west << " e: " << bounds_.east;
}

template<class Node>
QuadTreeNode<Node>::~QuadTreeNode() {
    if (children_ != nullptr) {
        for (const auto& child :* children_) {
            delete child;
        }
        delete children_;
        children_ = nullptr;
    }
}

template<class Node>
bool QuadTreeNode<Node>::hasChildren() {
    return children_ != nullptr;
}

template<class Node>
void QuadTreeNode<Node>::split() {
    if ((std::abs(bounds_.north - bounds_.south) < minSize_) &&
        (std::abs(bounds_.east - bounds_.west) < minSize_) ) {
        LOG(UtilLib::logDEBUG4) << "below min size therefore no split";

        return;
    }

    LOG(UtilLib::logDEBUG4) << "split executed";

    double nsHalf =
        static_cast<double>(bounds_.north -
                            (bounds_.north - bounds_.south) / 2.0);
    double ewHalf =
        static_cast<double>(bounds_.east - (bounds_.east - bounds_.west) / 2.0);

    children_ = new std::array<QuadTreeNode*, 4>;

    (*children_)[NORTHWEST] =
        new QuadTreeNode(Rectangle(bounds_.north, nsHalf, bounds_.west,
                        ewHalf), maxItems_);
    (*children_)[NORTHEAST] =
        new QuadTreeNode(Rectangle(bounds_.north, nsHalf, ewHalf,
                        bounds_.east), maxItems_);
    (*children_)[SOUTHEAST] =
        new QuadTreeNode(Rectangle(nsHalf, bounds_.south, ewHalf,
                        bounds_.east), maxItems_);
    (*children_)[SOUTHWEST] =
        new QuadTreeNode(Rectangle(nsHalf, bounds_.south, bounds_.west,
                        ewHalf), maxItems_);

    for (auto item : items_) {
        put(item);
    }
    items_.clear();
}

template<class Node>
QuadTreeNode<Node>* QuadTreeNode<Node>::getChild(
        double x,
        double y)
{
    if (bounds_.pointWithinBounds(x, y))
    {
        if (children_ != nullptr) {
            for (QuadTreeNode<Node>* child :* children_)
            {
                if (child->bounds_.pointWithinBounds(x, y))
                    return child->getChild(x, y);
            }
        } else {
            return this;  // no children, x, y here...
        }
    }
    return nullptr;
}

template<class Node>
bool QuadTreeNode<Node>::put(std::shared_ptr<Node> pt) {
    if (children_ == nullptr) {
        items_.push_back(pt);
        if (items_.size() > maxItems_)
            split();

        return true;
    } else {
        QuadTreeNode* node = getChild(pt->getXPos(), pt->getYPos());
        if (node != nullptr) {
            return node->put(pt);
        }
    }
    return false;
}

template<class Node>
std::vector<std::shared_ptr<Node> >& QuadTreeNode<Node>::get(
        const Rectangle& rect,
        std::vector<std::shared_ptr<Node> >& vector) {
    if (children_ == nullptr) {
        for (std::shared_ptr<Node> pt : items_) {
            if (rect.pointWithinBounds(pt->getXPos(), pt->getYPos())) {
                vector.push_back(pt);
                LOG(UtilLib::logDEBUG4) << "point added: " << pt->getXPos() <<
                " " << pt->getYPos();
            }
        }
    } else {
        for (QuadTreeNode* child :* children_) {
            if (child->bounds_.within(rect)) {
                LOG(UtilLib::logDEBUG4) << "went to children";
                child->get(rect, vector);
            }
        }
    }
    return vector;
}
}   // end namepace
}  // end namepace

#endif  // UTILLIB_GEOMETRY_QUADTREENODE_HPP
