/* Copyright (c) 2013 Simon Tanaka <tanakas"at"gmx"dot"ch>
 *               2014 Simon Tanaka <tanakas"at"gmx"dot"ch>
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
#ifndef FASTNEIGHBORLIST_HPP
#define FASTNEIGHBORLIST_HPP

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <cassert>

namespace UtilLib {
namespace geometry {

namespace {
/**
 * @brief The index struct
 *
 */
struct listindex {
    /**
     * @brief x The x index.
     */
    unsigned int x;

    /**
     * @brief y The y index.
     */
    unsigned int y;
};

/**
 * @brief distance Helper function to computes the Euclidian distance.
 * @param x1 The x coordinate of point 1.
 * @param y1 The y coordinate of point 1.
 * @param x2 The x coordinate of point 2.
 * @param y2 The y coordinate of point 2.
 * @return The Euclidian distance.
 */
inline double distance( const double x1,
                        const double y1,
                        const double x2,
                        const double y2) {
    return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}
}

//##########################################################################
// declaration:
//##########################################################################
/**
 * @brief The fastneighborlist class is a class template putting objects of type T into a celllist data structur for efficient range queries.
 * @attention The elements of type T must expose ->getPosX(), ->getPosY(), ->getDomainIdOfAdjacentConnections.
 */
template <typename T>
class fastneighborlist {
public:
    /**
     * @brief fastneighborlist Constructor
     * @param LX The x domain size.
     * @param LY The y domain size.
     * @param NX The number of bins in x direction.
     * @param NY The number of bins in y direction.
     */
    fastneighborlist(const double LX,
                     const double LY,
                     const unsigned int NX,
                     const unsigned int NY);

    /**
     * @brief addElement Adds an element into the fastneighborlist data structure.
     * @param element The element to add.
     */
    void addElement(T element);

    /**
     * @brief removeElement Removes an element from the fastneighborlist data structure.
     * @param element The element which shall be removed.
     * @return true if found+removed; false otherwise
     */
    bool removeElement(T element);

    /**
     * @brief Completely rebuilds the fastneighborlist. T must expose getXPos(), getYPos()
     * @param elementmap The map containing the elements{GeometryNodeID, std::shared_ptr<nodes::GeometryNode>}
     */
    void reset(std::map<unsigned int,T> elementmap);

    /**
     * @brief range query, returns all elements wich are within the radius.
     * @param queryresult vector reference to store the result in. will be emptied beforehand.
     * @param posx The x coordinate.
     * @param posy The ycoordinate.
     * @param radius The radius to search within.
     */
    void getGeometryNodesWithinRadius(  std::vector<T>& queryresult,
                                        const double posx,
                                        const double posy,
                                        const double radius) const;

    /**
     * @brief range query, but only nodes with domainID different from avoidDomainID
     * @param queryresult vector reference to store the result in. will be emptied beforehand.
     * @param posx The x coordinate.
     * @param posy The ycoordinate.
     * @param radius The radius to search within.
     * @param avoidDomainID The domainID to be avoided when returning *GeometryNode*s.
     */
    void getGeometryNodesWithinRadiusWithAvoidance( std::vector<T>& queryresult,
                                                    const double posx,
                                                    const double posy,
                                                    const double radius,
                                                    const unsigned int avoidDomainID) const;
    /**
     * @brief return closest *GeometryNode*, but only nodes with domainID different from avoidDomainID
     * @param queryresult vector reference to store the result in. will be emptied beforehand.
     * @param posx The x coordinate.
     * @param posy The ycoordinate.
     * @param radius The radius to search within.
     * @param avoidDomainID The domainID to be avoided when returning *GeometryNode*s.
     * @return The vector with 0 or 1 *GeometryNode*
     */
    void getGeometryNodesWithinRadiusWithAvoidanceClosest(  std::vector<T>& queryresult,
                                                            const double posx,
                                                            const double posy,
                                                            const double radius,
                                                            const unsigned int avoidDomainID) const;

    /**
     * @brief displayContent Displays all the content in form [xindex,yindex]: (posx,posy) ...
     */
    void displayContent() const;

private:
    /**
     * @brief removeElementAtIndex Just checks the bin at index.
     * @param element The element to remove.
     * @param index The index of the bin to look at.
     * @return Number of found+removed elements.
     */
    unsigned int removeElementAtIndex(T element, listindex index);

    /**
     * @brief removeElementBruteForce Traverses through entire list.
     * @param element The element to remove.
     * @return Number of found+removed elements.
     */
    unsigned int removeElementBruteForce(T element);

    /**
     * @brief appendContentOfCell Adds all the content of cell (i,j) to vec.
     * @param vec The vector to add the T objects to.
     * @param j The y index of the cell.
     * @param i The x index of the cell.
     */
    void appendContentOfCell(std::vector<T> &vec,
                             unsigned int j,
                             unsigned int i) const;

    /**
     * @brief appendContentOfNeighboringCells Adds all the content of cells within radius to queryresult.
     * @param queryresult The vector to add the T objects to.
     * @param posx The x position.
     * @param posy The y position.
     * @param radius The radius.
     */
    void appendContentOfNeighboringCells(std::vector<T>& queryresult,
                                         const double posx,
                                         const double posy,
                                         const double radius) const;

    /**
     * @brief getIndex Computes the cell index.
     * @param posx The x position.
     * @param posy The y position.
     * @return An index struct with the indices of the cell.
     */
    inline listindex getIndex(double posx,
                          double posy) const;

    /**
     * @brief clearList Resets the content of the list, but does not resize.
     */
    void clearList(void);

    /**
     * @brief BinSizeX_ The bin size in x direction.
     */
    const double BinSizeX_;

    /**
     * @brief BinSizeY_ The bin size in y direction.
     */
    const double BinSizeY_;

    /**
     * @brief list_ Contains a vector of T objects in a 2D vector. Indexing: [y][x]
     */
    std::vector< std::vector< std::vector< T > > > list_;
};


//##########################################################################
// implementation:
//##########################################################################
template<typename T>
fastneighborlist<T>::fastneighborlist(const double LX,
                                      const double LY,
                                      const unsigned int NX,
                                      const unsigned int NY) :  BinSizeX_(double(LX)/double(NX)),
                                                                BinSizeY_(double(LY)/double(NY))
{
    assert(LX > 0.0);
    assert(LY > 0.0);
    assert(NX > 0);
    assert(NY > 0);

    this->list_.resize(NY);
    for (auto &it : this->list_) {
        it.resize(NX);
    }
}

template<typename T>
void fastneighborlist<T>::addElement(T element)
{
    listindex ind = this->getIndex(element->getXPos(), element->getYPos());
    assert(ind.y < this->list_.size());
    assert(ind.x < this->list_[0].size());
    this->list_[ind.y][ind.x].push_back(element);
}

template<typename T>
bool fastneighborlist<T>::removeElement(T element)
{

    unsigned int count(0);
    listindex ind = this->getIndex(element->getXPos(),element->getYPos());
    listindex tempind;

    // step 1: first try the bin according to the position of element:
    count = this->removeElementAtIndex(element,ind);
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2a: check neighboring bin to the left:
    if (ind.x > 0) {
        tempind.x = ind.x - 1;
        tempind.y = ind.y;
        count = this->removeElementAtIndex(element,tempind);
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2b: check neighboring bin to the right:
    if (ind.x < this->list_[0].size()-1) {
        tempind.x = ind.x + 1;
        tempind.y = ind.y;
        count = this->removeElementAtIndex(element,tempind);
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2c: check neighboring bin to the bottom:
    if (ind.y > 0) {
        tempind.x = ind.x;
        tempind.y = ind.y - 1;
        count = this->removeElementAtIndex(element,tempind);
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2d: check neighboring bin to the top:
    if (ind.y < this->list_.size() - 1) {
        tempind.x = ind.x;
        tempind.y = ind.y + 1;
        count = this->removeElementAtIndex(element,tempind);
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2e: check neighboring bin to the bottom left:
    if (ind.x > 0) {
        tempind.x = ind.x - 1;
        if (ind.y > 0) {
            tempind.y = ind.y - 1;
            count = this->removeElementAtIndex(element,tempind);
        }
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2f: check neighboring bin to the bottom right:
    if (ind.x < this->list_[0].size() - 1) {
        tempind.x = ind.x + 1;
        if (ind.y > 0) {
            tempind.y = ind.y - 1;
            count = this->removeElementAtIndex(element,tempind);
        }
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2g: check neighboring bin to the top left:
    if (ind.x > 0) {
        tempind.x = ind.x - 1;
        if (ind.y < this->list_.size() - 1) {
            tempind.y = ind.y + 1;
            count = this->removeElementAtIndex(element,tempind);
        }
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 2h: check neighboring bin to the top right:
    if (ind.x < this->list_[0].size() - 1) {
        tempind.x = ind.x + 1;
        if (ind.y < this->list_.size() - 1) {
            tempind.y = ind.y + 1;
            count = this->removeElementAtIndex(element,tempind);
        }
    }
    assert( (count==0) || (count==1) );
    if (count==1) {return true;}

    // step 3: if still not found, screen brute force for the element:
    unsigned int num = this->removeElementBruteForce(element);
    assert( (num==0) || (num==1) );
    return (bool) num;
}

template<typename T>
unsigned int fastneighborlist<T>::removeElementAtIndex(T element, listindex index) {
    unsigned int count(0);

    // erase-remove idiom using lambda:
    this->list_[index.y][index.x].erase( std::remove_if(    this->list_[index.y][index.x].begin(),
                                                            this->list_[index.y][index.x].end(),
                                                            [element,&count](T node){
                                                            if (node == element) {
                                                                count++;
                                                                return true;
                                                            }
                                                            return false;
                                                            }),
                                                            this->list_[index.y][index.x].end()
                                                            );
    return count;
}

template<typename T>
unsigned int fastneighborlist<T>::removeElementBruteForce(T element)
{
    unsigned int count(0);
    listindex ind;
    const size_t sizeX = this->list_[0].size();

    for (size_t iy = 0;
         iy < this->list_.size();
         ++iy) {
        ind.y = iy;
        for (size_t ix = 0;
             ix < sizeX;
             ++ix) {
            ind.x = ix;
            count = this->removeElementAtIndex(element,ind);
        }
    }
    assert( (count==0) || (count==1) );
    return count;
}

template<typename T>
void fastneighborlist<T>::reset(std::map<unsigned int, T> elementmap)
{
    this->clearList();

    for (auto& it : elementmap) {
        this->addElement(it.second);
    }
}

template<typename T>
void fastneighborlist<T>::getGeometryNodesWithinRadius( std::vector<T>& queryresult,
                                                        const double posx,
                                                        const double posy,
                                                        const double radius) const
{
    queryresult.clear();
    this->appendContentOfNeighboringCells(queryresult, posx, posy, radius);

    // erase-remove idiom using lambda:
    queryresult.erase( std::remove_if(  queryresult.begin(),
                                        queryresult.end(),
                                        [posx,posy,radius](T node){
                                        if (distance(posx,posy,node->getXPos(),node->getYPos()) > radius) {
                                            return true;
                                        }
                                        return false;
                                        }),
                                        queryresult.end()
                       );
}

template<typename T>
void fastneighborlist<T>::getGeometryNodesWithinRadiusWithAvoidance(    std::vector<T> &queryresult,
                                                                        const double posx,
                                                                        const double posy,
                                                                        const double radius,
                                                                        const unsigned int avoidDomainID) const
{
    this->getGeometryNodesWithinRadius(queryresult, posx, posy, radius);

    // erase-remove idiom using lambda expression:
    queryresult.erase( std::remove_if(  queryresult.begin(),
                                        queryresult.end(),
                                        [avoidDomainID](T node) {
                                        if (node->getDomainIdOfAdjacentConnections() == avoidDomainID) {
                                            return true;
                                        }
                                        return false;
                                        }),
                                        queryresult.end()
            );
}
template<typename T>
void fastneighborlist<T>::getGeometryNodesWithinRadiusWithAvoidanceClosest( std::vector<T> &queryresult,
                                                                            const double posx,
                                                                            const double posy,
                                                                            const double radius,
                                                                            const unsigned int avoidDomainID) const
{
    this->getGeometryNodesWithinRadiusWithAvoidance(queryresult, posx, posy, radius, avoidDomainID);

    // find the minimum distance *GeometryNode* using lambda expression:
    if (queryresult.size() > 1) {
        auto min_it = std::min_element( queryresult.begin(),
                                        queryresult.end(),
                                        [posx,posy] (T node1, T node2) {
                                        return distance(posx,posy,node1->getXPos(),node1->getYPos()) < distance(posx,posy,node2->getXPos(),node2->getYPos());
                                        }
                                        );

        queryresult.clear();
        queryresult.push_back(*min_it);
    }
}

template<typename T>
void fastneighborlist<T>::displayContent() const
{
    for (auto ity=0; ity<this->list_.size(); ++ity) {
        for (auto itx=0; itx<this->list_[0].size(); ++itx) {
            std::cout << "[x=" << itx << ", y=" << ity << "]" << "\t";

            for (auto it : this->list_[ity][itx]) {
                std::cout << "(" << it->getXPos() << "," << it->getYPos() << ")" << "\t";
            }
            std::cout<<"\n";
        }
    }
    std::cout<<"\n";
}

template<typename T>
void fastneighborlist<T>::appendContentOfCell(std::vector<T> &vec,
                                              unsigned int j,
                                              unsigned int i) const {
    vec.insert(vec.end(), this->list_[j][i].begin(), this->list_[j][i].end());
}

template<typename T>
void fastneighborlist<T>::appendContentOfNeighboringCells(std::vector<T>& queryresult,
                                                          const double posx,
                                                          const double posy,
                                                          const double radius) const {
    // bounding box:
    listindex minind = this->getIndex(posx-radius,posy-radius);
    listindex maxind = this->getIndex(posx+radius,posy+radius);

    for (unsigned int j=minind.y; j<maxind.y+1; ++j) {
        for (unsigned int i=minind.x; i<maxind.x+1; ++i) {
            this->appendContentOfCell(queryresult,j,i);
        }
    }
}

template<typename T>
listindex fastneighborlist<T>::getIndex(double posx,
                                    double posy) const
{
    listindex ind;
    if (posx < 0.0) posx = 0.0;
    if (posy < 0.0) posy = 0.0;
    ind.x = std::floor(posx/this->BinSizeX_);
    ind.y = std::floor(posy/this->BinSizeY_);
    if (ind.x >= this->list_[0].size()) ind.x = this->list_[0].size()-1;
    if (ind.y >= this->list_.size()) ind.y = this->list_.size()-1;
    return ind;
}

template<typename T>
void fastneighborlist<T>::clearList()
{
    for (auto& ity : this->list_) {
        for (auto& itx : ity) {
            itx.clear();
        }
    }
}


}
}



#endif // FASTNEIGHBORLIST_HPP
