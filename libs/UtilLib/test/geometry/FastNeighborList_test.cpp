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
#define BOOST_TEST_DYN_LINK // main is generated automatically
#define BOOST_TEST_MODULE fastneighborhood_test

#include <boost/test/unit_test.hpp>

#include <iostream>
#include <memory>
#include <map>

#define private public
#define protected public
#include <UtilLib/include/geometry/FastNeighborList.hpp>
#undef protected
#undef private

class GeometryNode {
public:
    GeometryNode(double xpos,
                 double ypos) : xpos_(xpos),
                                ypos_(ypos),
                                domainid_(1) {
    }

    double getXPos() {
        return this->xpos_;
    }

    double getYPos() {
        return this->ypos_;
    }

    void setXPos(double xpos) {
        this->xpos_ = xpos;
    }

    void setYPos(double ypos) {
        this->ypos_ = ypos;
    }

    unsigned int getDomainIdOfAdjacentConnections() {
        return this->domainid_;
    }

    void setDomainId(unsigned int domainid) {
        this->domainid_ = domainid;
    }

private:
    double xpos_;
    double ypos_;
    unsigned int domainid_;
};

typedef UtilLib::geometry::fastneighborlist<std::shared_ptr<GeometryNode> > mylist;


BOOST_AUTO_TEST_SUITE( fastneighborlist_test )

BOOST_AUTO_TEST_CASE( addelement_test ) {
    const double LX = 4.0;
    const double LY = 4.0;
    unsigned int NX = 4;
    unsigned int NY = 8;

    mylist LIST(LX,LY,NX,NY);

    LIST.clearList();
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(0.0,0.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.0,1.2)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.1,3.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.2,3.3)));

    BOOST_CHECK_EQUAL(LIST.list_[0][0][0]->getXPos(), 0.0);
    BOOST_CHECK_EQUAL(LIST.list_[0][0][0]->getYPos(), 0.0);

    BOOST_CHECK_EQUAL(LIST.list_[2][1][0]->getXPos(), 1.0);
    BOOST_CHECK_EQUAL(LIST.list_[2][1][0]->getYPos(), 1.2);

    BOOST_CHECK_EQUAL(LIST.list_[6][1][0]->getXPos(), 1.1);
    BOOST_CHECK_EQUAL(LIST.list_[6][1][0]->getYPos(), 3.0);

    BOOST_CHECK_EQUAL(LIST.list_[6][1][1]->getXPos(), 1.2);
    BOOST_CHECK_EQUAL(LIST.list_[6][1][1]->getYPos(), 3.3);
}


BOOST_AUTO_TEST_CASE( removeelement_test ) {
    const double LX = 4.0;
    const double LY = 4.0;
    unsigned int NX = 4;
    unsigned int NY = 8;

    mylist LIST(LX,LY,NX,NY);

    LIST.clearList();
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(0.0,0.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.0,1.2)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.1,3.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.2,3.3)));

    auto RM = std::make_shared<GeometryNode>(GeometryNode(1.2,3.4));
    LIST.addElement(RM);

    BOOST_CHECK_EQUAL(LIST.list_[6][1][2]->getXPos(), 1.2);
    BOOST_CHECK_EQUAL(LIST.list_[6][1][2]->getYPos(), 3.4);
    BOOST_CHECK_EQUAL(LIST.list_[6][1].size(), 3);

    LIST.removeElement(RM);
    BOOST_CHECK_EQUAL(LIST.list_[6][1].size(), 2);


    // test whether an externally moved *GeometryNode* can be removed:
    LIST.clearList();
    std::vector<std::shared_ptr<GeometryNode>> nodelist;
    nodelist.push_back(std::make_shared<GeometryNode>(GeometryNode(1.0,1.2)));
    nodelist.push_back(std::make_shared<GeometryNode>(GeometryNode(1.1,3.0)));
    nodelist.push_back(std::make_shared<GeometryNode>(GeometryNode(0.0,3.0)));
    nodelist.push_back(std::make_shared<GeometryNode>(GeometryNode(3.1,0.0)));
    nodelist.push_back(std::make_shared<GeometryNode>(GeometryNode(3.0,1.0)));

    for (auto it : nodelist) {
        LIST.addElement(it);
    }
    BOOST_CHECK_EQUAL( LIST.list_[2][1][0]->getXPos() , 1.0 );
    BOOST_CHECK_EQUAL( LIST.list_[2][1][0]->getYPos() , 1.2 );

    nodelist[0]->setXPos(6.0);

    BOOST_CHECK_EQUAL( LIST.list_[2][1][0]->getXPos() , 6.0 );
    BOOST_CHECK_EQUAL( LIST.list_[2][1][0]->getYPos() , 1.2 );

    LIST.removeElement(nodelist[0]);

    BOOST_CHECK_EQUAL( LIST.list_[2][1].size() , 0 );

    LIST.addElement(nodelist[0]);

    BOOST_CHECK_EQUAL( LIST.list_[2][3].size() , 2 );
    BOOST_CHECK_EQUAL( LIST.list_[2][3][1]->getXPos() , 6.0 );
    BOOST_CHECK_EQUAL( LIST.list_[2][3][1]->getYPos() , 1.2 );
}

BOOST_AUTO_TEST_CASE( reset_test ) {
    const double LX = 4.0;
    const double LY = 4.0;
    unsigned int NX = 4;
    unsigned int NY = 8;

    mylist LIST(LX,LY,NX,NY);

    LIST.clearList();
    std::map<unsigned int, std::shared_ptr<GeometryNode> > elementmap;
    elementmap[0] = std::make_shared<GeometryNode>(GeometryNode(0.0,0.0));
    elementmap[1] = std::make_shared<GeometryNode>(GeometryNode(1.0,1.2));
    elementmap[5] = std::make_shared<GeometryNode>(GeometryNode(1.1,3.0));
    elementmap[9] = std::make_shared<GeometryNode>(GeometryNode(1.2,3.3));
    LIST.reset(elementmap);

    BOOST_CHECK_EQUAL(LIST.list_[0][0][0]->getXPos(), 0.0);
    BOOST_CHECK_EQUAL(LIST.list_[0][0][0]->getYPos(), 0.0);

    BOOST_CHECK_EQUAL(LIST.list_[2][1][0]->getXPos(), 1.0);
    BOOST_CHECK_EQUAL(LIST.list_[2][1][0]->getYPos(), 1.2);

    BOOST_CHECK_EQUAL(LIST.list_[6][1][0]->getXPos(), 1.1);
    BOOST_CHECK_EQUAL(LIST.list_[6][1][0]->getYPos(), 3.0);

    BOOST_CHECK_EQUAL(LIST.list_[6][1][1]->getXPos(), 1.2);
    BOOST_CHECK_EQUAL(LIST.list_[6][1][1]->getYPos(), 3.3);
}

BOOST_AUTO_TEST_CASE( displayPointsWithinRadius_test ) {
    const double LX = 4.0;
    const double LY = 4.0;
    unsigned int NX = 4;
    unsigned int NY = 8;

    std::vector<std::shared_ptr<GeometryNode> > queryresult;
    mylist LIST(LX,LY,NX,NY);

    LIST.clearList();
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(0.0,0.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.0,1.2)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.1,3.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.2,3.3)));

    LIST.getGeometryNodesWithinRadius(queryresult,1.0,0.0,1.0);

    BOOST_CHECK_EQUAL(queryresult.size(),1);
    BOOST_CHECK_EQUAL(queryresult[0]->getXPos(), 0.0);
    BOOST_CHECK_EQUAL(queryresult[0]->getYPos(), 0.0);

    LIST.getGeometryNodesWithinRadius(queryresult,1.0,0.0,10.0);

    BOOST_CHECK_EQUAL(queryresult.size(),4);
}

BOOST_AUTO_TEST_CASE( displayPointsWithinRadiusWithAvoidance_test ) {
    const double LX = 4.0;
    const double LY = 4.0;
    unsigned int NX = 4;
    unsigned int NY = 8;

    std::vector<std::shared_ptr<GeometryNode> > queryresult;
    mylist LIST(LX,LY,NX,NY);

    LIST.clearList();
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(0.0,0.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.0,1.2)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.1,3.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.2,3.3)));

    LIST.getGeometryNodesWithinRadiusWithAvoidance(queryresult,1.0,0.0,10.0,1);

    BOOST_CHECK_EQUAL(queryresult.size(),0);

    LIST.getGeometryNodesWithinRadiusWithAvoidance(queryresult,1.0,0.0,10.0,0);

    BOOST_CHECK_EQUAL(queryresult.size(),4);
}

BOOST_AUTO_TEST_CASE( displayPointsWithinRadiusWithAvoidanceClosest_test ) {
    const double LX = 4.0;
    const double LY = 4.0;
    unsigned int NX = 4;
    unsigned int NY = 8;

    std::vector<std::shared_ptr<GeometryNode> > queryresult;
    mylist LIST(LX,LY,NX,NY);

    LIST.clearList();
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(0.0,0.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.0,1.2)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.1,3.0)));
    LIST.addElement(std::make_shared<GeometryNode>(GeometryNode(1.2,3.3)));

    LIST.getGeometryNodesWithinRadiusWithAvoidanceClosest(queryresult,1.0,0.0,10.0,1);

    BOOST_CHECK_EQUAL(queryresult.size(),0);

    LIST.getGeometryNodesWithinRadiusWithAvoidanceClosest(queryresult,1.2,3.0,10.0,0);

    BOOST_CHECK_EQUAL(queryresult.size(),1);
    BOOST_CHECK_EQUAL(queryresult[0]->getXPos(), 1.1);
    BOOST_CHECK_EQUAL(queryresult[0]->getYPos(), 3.0);
}

BOOST_AUTO_TEST_SUITE_END()
