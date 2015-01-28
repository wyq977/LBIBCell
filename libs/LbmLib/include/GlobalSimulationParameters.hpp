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
#ifndef GLOBALSIMULATIONPARAMETERS_HPP
#define GLOBALSIMULATIONPARAMETERS_HPP

#include <UtilLib/include/Singleton.hpp>
#include <map>
#include <string>
#include <vector>

namespace LbmLib {
/**
 * @brief The class which stores all parameters.
 * Do not instantiate this class but use the singleton provided below
 */
class GlobalSimulationParameters_ {
 public:
    /**
     * @brief the class that stores all parameters related to the simulation
     */
    GlobalSimulationParameters_();

    /**
     * @brief loads the parameters from the file
     *
     * @param fileName the filename
     */
    void loadGlobalSimulationParameters(const std::string& fileName);

    /**
     * @brief writes all parameters to the file
     *
     * @param fileName the filename
     */
    void writeGlobalSimulationParameters(const std::string& fileName) const;

    /**
     * @brief prints the parameters to the standard output
     */
    void printParameters() const;

    /**
     * @brief getter for the current iterations
     *
     * @return the current iterations
     */
    unsigned int getCurrentIteration() const {
        return currentIteration_;
    }

    /**
     * @brief getter for the number of iterations the simulation should be
     * executed
     * @return the number iterations
     */
    unsigned int getIterations() const {
        return iterations_;
    }

    /**
     * @brief getter for the report steps
     *
     * @return the number of report steps
     */
    unsigned int getReportSteps() const {
        return reportSteps_;
    }

    /**
     * @brief setReportSteps
     * @param steps
     */
    void setReportSteps(unsigned int steps) {
        this->reportSteps_ = steps;
    }

    /**
     * @brief getter for the tau of the fluid
     * @return the tau of the fluid
     */
    double getTauFluid() const {
        return tauFluid_;
    }

    /**
     * @brief getter for the size x
     *
     * @return the size x of the simulation
     */
    unsigned int getSizeX() const {
        return sizeX_;
    }

    /**
     * @brief getter for the size y
     *
     * @return the size y of the simulaton
     */
    unsigned int getSizeY() const {
        return sizeY_;
    }

    /**
     * @brief getter for the cdeSolvers and their tau's
     *
     * @return list of the mapping
     */
    const std::map<std::string, double>& getCdeSolvers() const {
        return cdeSolvers_;
    }

    /**
     * @brief increment operator to increase the current iterations
     * @param integer the incremented
     * @return the number of current iterations
     */
    unsigned int operator++(int integer) {
        return currentIteration_++;
    }

 private:
    /**
     * @brief the current number of iterations
     */
    unsigned int currentIteration_;

    /**
     * @brief the number of iterations the simulation should be executed
     */
    unsigned int iterations_;

    /**
     * @brief the report Step interval
     */
    unsigned int reportSteps_;

    /**
     * @brief the x size of the simulation
     */
    unsigned int sizeX_;

    /**
     * @brief the y size of the simulation
     */
    unsigned int sizeY_;

    /**
     * @brief the tau value of the fluid
     */
    double tauFluid_;

    /**
     * @brief the map of the cde solvers and their tau values
     */
    std::map<std::string, double> cdeSolvers_;
};


/**
 * @brief Singleton class of the parameters
 */
typedef UtilLib::Singleton<GlobalSimulationParameters_>
globalSimulationParametersSingleton_;

/**
 * @brief Accessor for the parameter class
 *
 * @return the unique instance of the parameters
 */
static GlobalSimulationParameters_& Parameters =
    globalSimulationParametersSingleton_::instance();
}  //  end namespace

#endif  // GLOBALSIMULATIONPARAMETERS_HPP
