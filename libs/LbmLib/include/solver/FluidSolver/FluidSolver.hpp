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
#ifndef FLUIDSOLVER_HPP
#define FLUIDSOLVER_HPP

#include <LbmLib/include/Field.hpp>
#include <LbmLib/include/solver/AbstractSolver.hpp>
#include <LbmLib/include/solver/FluidSolver/BaseForceModel.hpp>
#include <LbmLib/include/solver/FluidSolver/GuoZhengShi2002ForceModel.hpp>
#include <LbmLib/include/solver/FluidSolver/Luo1993ForceModel.hpp>
#include <array>
#include <string>
namespace LbmLib {
namespace nodes {
class PhysicalNode;
}

namespace solver {
/**
 * @brief the Fluid Solver which solves the D2Q9 LBGK
 */
class FluidSolver : public AbstractSolver, private /*GuoZhengShi2002ForceModel*/ Luo1993ForceModel {
 public:
    /**
     * @brief FluidSolver Initialises the fluid solver
     * @param physicalNode The physical node which is the parent of this solver
     */
    explicit FluidSolver(const nodes::PhysicalNode& physicalNode);
    /**
     * @brief  ~FluidSolver Destructor non virtual to avoid inheritance
     */
    ~FluidSolver() {}

    /**
     * @brief loads the solver from the file
     * @param stream the stream where the solver is loaded from
     */
    virtual void loadSolver(std::stringstream* const stream);

    /**
     * @brief writes the solver to the file
     * @param stream the stream where the solver is written to
     */
    virtual void writeSolver(std::ostream* const stream);

    /**
     * @brief collide The collision step of the LBM
     */
    virtual void collide();

    /**
     * @brief advect The advect step of the LBM
     */
    virtual void advect();

    /**
     * @brief accessDistribution Access to the distribution
     * @param dir the direction where the Distribution is wanted
     * @return A Reference to the Distribution
     */
    virtual double& accessDistribution(const Direction& dir);

    /**
     * @brief Rescales all distributions by a factor.
     * @param factor The rescaling factor.
     */
    virtual void rescaleDistributions(const double factor);

    /**
     * @brief getRho Calculates the Rho
     * @return returns the rho
     */
    double getRho() const;

    /**
     * @brief getVelocity Returns the current velocity of the fluid
     * @return The velocity of the fluid
     */
    const Field<double>& getVelocity() const;

    /**
     * @brief setVelocity Sets the velocity of this fluid Algorithm. Should only be used for initialisation.
     * @param velocity The velocity
     */
    void setVelocity(Field<double> velocity);

    /**
     * @brief initSolver Use this to initalise the solver
     */
    virtual void initSolver();

    /**
     * @brief  adds f to the current force
     * @post force_ = force_ + f
     * @param f the added force
     */
    void addForce(Field<double> f);

    /**
     * @brief resets the force on this fluid solver to 0
     * @post force_ = 0,0
     */
    void resetForce();

    /**
     * @brief addMass The mass which is added to this fluid solver
     * @param mass The mass to add
     */
    void addMass(double mass);

 private:
    /**
     * @brief initWithVelocity Inits the fluid solver with a certain speed.
     */
    void initWithVelocity();

    /**
     * @brief localSwap Executes the local swap inpreparation of the advect step
     */
    void localSwap();

    /**
     * @brief physicalNode_ The physical Node which owns this solver
     */
    const nodes::PhysicalNode& physicalNode_;
    /**
     * @brief distributions_ The distributions of this solver
     */
    std::array<double, 9> distributions_;
    /**
     * @brief velocity_ Storage of the speed.
     */
    Field<double> velocity_;
    /**
     * @brief force_ Storage of the speed.
     */
    Field<double> force_;
    /**
     * @brief speedInit Variable to use velocity initialisation if setSpeed was called
     */
    bool velocityInit_;
    /**
     * @brief dirIter_ Iterator for the Directions
     */
    static const DirectionIterator dirIter_;
    /**
     * @brief The mass which is added to the rho
     */
    double rhoToAdd_;
};


}
}  // end namespace
#endif  // FLUIDSOLVER_HPP
