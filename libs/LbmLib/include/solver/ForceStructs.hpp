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
#ifndef FORCESTRUCTS_HPP
#define FORCESTRUCTS_HPP

#include <LbmLib/include/Field.hpp>
#include <LbmLib/include/nodes/EulerianPoint.hpp>
#include <LbmLib/include/nodes/GeometryNode.hpp>
#include <LbmLib/include/solver/ForceSolver.hpp>
#include <UtilLib/include/Exception.hpp>
#include <omp.h>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
namespace LbmLib {
namespace solver {



namespace ForceStructs{

//=================================================================================================================
/**
 * @brief a spring froce between 2 nodes
 * @attention The force is ony applied to id1. To get the reaction, add the force with switched ids.
 */
struct ForceType0 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the first node
     * @param id2 the id of the second node
     * @param k the spring constant
     * @param l0 the default distance
     */
    ForceType0(
            unsigned int id1,
            unsigned int id2,
            double k,
            double l0) : AbstractForceStruct(),
                         id1_(id1),
                         id2_(id2),
                         k_(k),
                         l0_(l0) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes)
    {
        try {
            const double f = k_ *
                (nodes::getDistance(*(nodes.at(this->id2_)),
                         *(nodes.at(this->id1_))) - this->l0_);

            nodes.at(this->id1_)->addForce(f *
                    (nodes.at(this->id2_)->getPos() - nodes.at(this->id1_)->getPos()) /
                    nodes::getDistance(*(nodes.at(this->id2_)), *(nodes.at(this->id1_))));
        }
        catch (...){
            lbm_fail("Could not compute and set the forces.");
        }
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << this->type << "\t" << this->id1_ << "\t" << this->id2_ << "\t" << this->k_ <<
        "\t" << this->l0_ << "\n";
    }

    unsigned int getType() const {
        return 0;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return this->id2_;
    }

 private:
    const unsigned int type = 0;  ///< the force type
    const unsigned int id1_;  ///< the id of the first node
    const unsigned int id2_;  ///< the id of the second node
    const double k_;  ///< the spring constant
    const double l0_;  ///< the default distance
};

/**
 * @brief loader of the force
 * @param stream the stream where the force is extracted from
 * @return the new force
 */
std::shared_ptr<ForceType0> loadForceType0(std::stringstream* const stream) {
    unsigned int type, id1, id2;
    double k, l0;
    (*stream) >> type >> id1 >> id2 >> k >> l0;
    return std::make_shared<ForceType0>(ForceType0(id1, id2, k, l0));
}

//=================================================================================================================
/**
 * @brief a spring force between a node and a certain point
 */
struct ForceType1 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the node where the force is applied
     * @param x the x position of the anchor
     * @param y the y position of the anchor
     * @param k the spring constant
     * @param l0 the default distance
     */
    ForceType1(
            unsigned int id1,
            double x,
            double y,
            double k,
            double l0) : AbstractForceStruct(),
                         id1_(id1),
                         x_(x),
                         y_(y),
                         k_(k),
                         l0_(l0) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes) {
        nodes::LagrangianPoint tempNode(x_, y_);
        const double f = k_ * (nodes::getDistance(tempNode, *(nodes.at(id1_))) - l0_);
        double distance = nodes::getDistance(tempNode, *(nodes.at(id1_)));
        if (distance != 0.0) {
           nodes.at(id1_)->addForce(f * (tempNode.getPos() - nodes.at(
                                              id1_)->getPos()) / distance);
        }
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << type << "\t" << id1_ << "\t" << x_ << '\t' <<
        y_ << "\t" << k_ << "\t" << l0_ << "\n";
    }

    unsigned int getType() const {
        return 1;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return 0;
    }

 private:
    const unsigned int type = 1;  ///< the type of the force struct
    const unsigned int id1_;  ///< the id of the node where the force is applied
    const double x_;  ///< the x position of the point
    const double y_;  ///< the y position of the point
    const double k_;  ///< the spring constant
    const double l0_;  ///< the default distance
};

/**
 * @brief loader of the force
 * @param stream the stream where the force is extracted from
 * @return the new force
 */
std::shared_ptr<ForceType1> loadForceType1(std::stringstream* const stream) {
    unsigned int type, id1;
    double x, y, k, l0;
    (*stream) >> type >> id1 >> x >> y >> k >> l0;
//    return new ForceType1(id1, x, y, k, l0);
    return std::make_shared<ForceType1>(ForceType1(id1, x, y, k, l0));
}

//=================================================================================================================
/**
 * @brief a free force on a node
 */
struct ForceType2 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the node the free force is applied
     * @param fx the free force in x direction
     * @param fy the free force in y direction
     */
    ForceType2(
            unsigned int id1,
            double fx,
            double fy) : AbstractForceStruct(),
                         id1_(id1),
                         fx_(fx),
                         fy_(fy) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes) {
        nodes.at(id1_)->addForce(Field<double>(fx_, fy_));
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << type << "\t" << id1_ << "\t" << fx_ << "\t" << fy_ << "\n";
    }

    unsigned int getType() const {
        return 2;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return 0;
    }

 private:
    const unsigned int type = 2;  ///< the type of the force struct
    const unsigned int id1_;  ///< the id of the node where the force is applied
    const double fx_;  ///< the force in x direction
    const double fy_;  ///< the force in y direction
};

/**
 * @brief loader of the force
 * @param stream the stream where the force is extracted from
 * @return the new force
 */
std::shared_ptr<ForceType2> loadForceType2(std::stringstream* const stream) {
    unsigned int type, id1;
    double fx, fy;
    (*stream) >> type >> id1 >> fx >> fy;
//    return new ForceType2(id1, fx, fy);
    return std::make_shared<ForceType2>(ForceType2(id1, fx, fy));
}

//=================================================================================================================
/**
 * @brief a spring force between a node and a horzontal slider (sliding at hight y)
 */
struct ForceType3 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the node where the force is applied
     * @param y The y position of the horizontal slider.
     * @param k the spring constant
     * @param l0 the default distance
     */
    ForceType3(
            unsigned int id1,
            double y,
            double k,
            double l0) : AbstractForceStruct(),
                         id1_(id1),
                         y_(y),
                         k_(k),
                         l0_(l0) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes) {
        nodes::LagrangianPoint tempNode(nodes.at(this->id1_)->getXPos(), y_);
        const double f = this->k_ * (nodes::getDistance(tempNode, *(nodes.at(this->id1_))) - this->l0_);
        Field<double> force;
        force.x = 0;
        if (nodes.at(this->id1_)->getYPos() < this->y_) { // get the sign right
            force.y = f;
        }
        else {
            force.y = -f;
        }
        nodes.at(this->id1_)->addForce(force);
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << type << "\t" << id1_ << "\t" <<
        y_ << "\t" << k_ << "\t" << l0_ << "\n";
    }

    unsigned int getType() const {
        return 3;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return 0;
    }

 private:
    const unsigned int type = 3;  ///< the type of the force struct
    const unsigned int id1_;  ///< the id of the node where the force is applied
    const double y_;  ///< the y position of the slider
    const double k_;  ///< the spring constant
    const double l0_;  ///< the default distance
};

/**
 * @brief loader of the force type 3
 * @param stream the stream where the force is extracted from
 * @return the new force struct
 */
std::shared_ptr<ForceType3> loadForceType3(std::stringstream* const stream) {
    unsigned int type, id1;
    double y, k, l0;
    (*stream) >> type >> id1 >> y >> k >> l0;
    return std::make_shared<ForceType3>(ForceType3(id1, y, k, l0));
}

//=================================================================================================================
/**
 * @brief a spring force between a node and a vertical slider (sliding at width x)
 */
struct ForceType4 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the node where the force is applied
     * @param x The x position of the vertical slider
     * @param k the spring constant
     * @param l0 the default distance
     */
    ForceType4(
            unsigned int id1,
            double x,
            double k,
            double l0) : AbstractForceStruct(),
                         id1_(id1),
                         x_(x),
                         k_(k),
                         l0_(l0) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes) {
        nodes::LagrangianPoint tempNode(x_, nodes.at(this->id1_)->getYPos());
        const double f = this->k_ * (nodes::getDistance(tempNode, *(nodes.at(this->id1_))) - this->l0_);
        Field<double> force;
        if (nodes.at(this->id1_)->getXPos() < this->x_) {
            force.x = f;
        }
        else {
            force.x = -f;
        }
        force.y = 0;
        nodes.at(this->id1_)->addForce(force);
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << type << "\t" << id1_ << "\t" <<
        x_ << "\t" << k_ << "\t" << l0_ << "\n";
    }

    unsigned int getType() const {
        return 4;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return 0;
    }

 private:
    const unsigned int type = 4;  ///< the type of the force struct
    const unsigned int id1_;  ///< the id of the node where the force is applied
    const double x_;  ///< the y position of the slider
    const double k_;  ///< the spring constant
    const double l0_;  ///< the default distance
};

/**
 * @brief loader of the force type 3
 * @param stream the stream where the force is extracted from
 * @return the new force struct
 */
std::shared_ptr<ForceType4> loadForceType4(std::stringstream* const stream) {
    unsigned int type, id1;
    double x, k, l0;
    (*stream) >> type >> id1 >> x >> k >> l0;
//    return new ForceType4(id1, x, k, l0);
    return std::make_shared<ForceType4>(ForceType4(id1, x, k, l0));
}

//=================================================================================================================
/**
 * @brief a constant force between 2 nodes
 * @attention The force is ony applied to id1. To get the reaction, add the force with switched ids.
 */
struct ForceType5 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the first node
     * @param id2 the id of the second node
     * @param f the constant force
     */
    ForceType5(
            unsigned int id1,
            unsigned int id2,
            double f) : AbstractForceStruct(),
                         id1_(id1),
                         id2_(id2),
                         f_(f) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes)
    {
        try {
            nodes.at(this->id1_)->addForce(this->f_ *
                    (nodes.at(this->id2_)->getPos() - nodes.at(this->id1_)->getPos()) /
                    nodes::getDistance(*(nodes.at(this->id2_)), *(nodes.at(this->id1_))));
        }
        catch (...){
            lbm_fail("Could not compute and set the forces.");
        }
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << this->type << "\t" << this->id1_ << "\t" << this->id2_ << "\t" << this->f_ << "\n";
    }

    unsigned int getType() const {
        return 5;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return this->id2_;
    }

 private:
    const unsigned int type = 5;  ///< the force type
    const unsigned int id1_;  ///< the id of the first node
    const unsigned int id2_;  ///< the id of the second node
    const double f_;  ///< the constant force
};

/**
 * @brief loader of the force
 * @param stream the stream where the force is extracted from
 * @return the new force
 */
std::shared_ptr<ForceType5> loadForceType5(std::stringstream* const stream) {
    unsigned int type, id1, id2;
    double f;
    (*stream) >> type >> id1 >> id2 >> f;
//    return new ForceType5(id1, id2, f);
    return std::make_shared<ForceType5>(ForceType5(id1, id2, f));
}

//=================================================================================================================
/**
 * @brief a constant force between 2 nodes
 * @attention The force is ony applied to id1. To get the reaction, add the force with switched ids.
 */
struct ForceType6 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the first node
     * @param id2 the id of the second node
     * @param f the constant force
     */
    ForceType6(
            unsigned int id1,
            unsigned int id2,
            double f) : AbstractForceStruct(),
                         id1_(id1),
                         id2_(id2),
                         f_(f) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes)
    {
        assert(nodes.find(this->id1_) != nodes.end());
        assert(nodes.find(this->id2_) != nodes.end());
        try {
            nodes.at(this->id1_)->addForce(this->f_ *
                    (nodes.at(this->id2_)->getPos() - nodes.at(this->id1_)->getPos()) /
                    nodes::getDistance(*(nodes.at(this->id2_)), *(nodes.at(this->id1_))));
        }
        catch (...){
            lbm_fail("Could not compute and set the forces.");
        }
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << this->type << "\t" << this->id1_ << "\t" << this->id2_ << "\t" << this->f_ << "\n";
    }

    unsigned int getType() const {
        return 6;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return this->id2_;
    }

 private:
    const unsigned int type = 6;  ///< the force type
    const unsigned int id1_;  ///< the id of the first node
    const unsigned int id2_;  ///< the id of the second node
    const double f_;  ///< the constant force
};

/**
 * @brief loader of the force
 * @param stream the stream where the force is extracted from
 * @return the new force
 */
std::shared_ptr<ForceType6> loadForceType6(std::stringstream* const stream) {
    unsigned int type, id1, id2;
    double f;
    (*stream) >> type >> id1 >> id2 >> f;
    return std::make_shared<ForceType6>(ForceType6(id1, id2, f));
}

//=================================================================================================================
/**
 * @brief a spring froce between 2 nodes
 * @attention The force is ony applied to id1. To get the reaction, add the force with switched ids.
 */
struct ForceType7 : public AbstractForceStruct {
    /**
     * @brief the constructor of this force
     * @param id1 the id of the first node
     * @param id2 the id of the second node
     * @param k the spring constant
     * @param l0 the default distance
     */
    ForceType7(
            unsigned int id1,
            unsigned int id2,
            double k,
            double l0) : AbstractForceStruct(),
                         id1_(id1),
                         id2_(id2),
                         k_(k),
                         l0_(l0) {}

    void calculateForce(const std::map<unsigned int,
                    std::shared_ptr<nodes::GeometryNode> >& nodes)
    {
        try {
            const double f = k_ *
                (nodes::getDistance(*(nodes.at(this->id2_)),
                         *(nodes.at(this->id1_))) - this->l0_);

            nodes.at(this->id1_)->addForce(f *
                    (nodes.at(this->id2_)->getPos() - nodes.at(this->id1_)->getPos()) /
                    nodes::getDistance(*(nodes.at(this->id2_)), *(nodes.at(this->id1_))));
        }
        catch (...){
            lbm_fail("Could not compute and set the forces.");
        }
    }

    void writeForceStruct(std::ofstream* const stream) const {
        (*stream) << this->type << "\t" << this->id1_ << "\t" << this->id2_ << "\t" << this->k_ <<
        "\t" << this->l0_ << "\n";
    }

    unsigned int getType() const {
        return 7;
    }

    unsigned int getPartnerGeometryNode(void) const {
        return this->id2_;
    }

 private:
    const unsigned int type = 7;  ///< the force type
    const unsigned int id1_;  ///< the id of the first node
    const unsigned int id2_;  ///< the id of the second node
    const double k_;  ///< the spring constant
    const double l0_;  ///< the default distance
};

/**
 * @brief loader of the force
 * @param stream the stream where the force is extracted from
 * @return the new force
 */
std::shared_ptr<ForceType7> loadForceType7(std::stringstream* const stream) {
    unsigned int type, id1, id2;
    double k, l0;
    (*stream) >> type >> id1 >> id2 >> k >> l0;
//    return new ForceType7(id1, id2, k, l0);
    return std::make_shared<ForceType7>(ForceType7(id1, id2, k, l0));
}




}       // end anonymous namespace

}
}

#endif // FORCESTRUCTS_HPP
