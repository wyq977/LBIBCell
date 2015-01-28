/* Copyright (c) 2013 David Sichau <mail"at"sichau"dot"eu>
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
#include <UtilLib/config.hpp>
#ifdef ENABLE_MPI
#include <boost/mpi/communicator.hpp>
#endif
#include <UtilLib/include/MPIProxy.hpp>
#define private public
#define protected public
#include <UtilLib/include/ProgressBar.hpp>
#undef protected
#undef private
#include <boost/test/minimal.hpp>
#include <cstring>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
using namespace boost::unit_test;
using namespace UtilLib;

void Wait(int Seconds) {
    clock_t WaitTime = clock() + Seconds * CLOCKS_PER_SEC;
    while (clock() < WaitTime) {}
}

void test_Constructor() {
    std::stringstream os;

    ProgressBar pb(100, 0.5, os);

    if (MPIProxy().getRank() == 0) {
        BOOST_CHECK(pb.expectedCount_ == 100);
    }
    BOOST_CHECK(pb.updateInterval_ == 0.5);

    if (MPIProxy().getRank() == 0) {
        std::stringstream tempStream;
        tempStream << "progress: " << std::setw(4) << std::setprecision(3) <<
            static_cast<float>(0) << "%   estimated time remaining:" <<
            std::setw(6) << std::setprecision(3) << "inf sec\n"
                   << "0%   10   20   30   40   50   60   70   80   90   100%\n"
                   << "|----|----|----|----|----|----|----|----|----|----|"
                   << std::endl;
        BOOST_CHECK(os.str() == tempStream.str());
        pb++;
        pb++;
        pb++;
        pb++;
        std::string s = os.str();
        BOOST_CHECK(std::count(s.begin(), s.end(), '*') == 3);
        ++pb;
        ++pb;
        ++pb;
        s = os.str();
        BOOST_CHECK(std::count(s.begin(), s.end(), '*') == 4);
        pb += 50;
        s = os.str();
        BOOST_CHECK(std::count(s.begin(), s.end(), '*') == 28);
    }
}

void test_timeUpdate() {
    int count = 2;
    std::stringstream os;

    UtilLib::ProgressBar pb(100000, 0.5, os);
    for (int i = 0; i < count; i++) {
        Wait(1);
        pb++;
    }
    std::string s = os.str();
    BOOST_CHECK(s.find_first_of("progress: 0.002%") != std::string::npos);
    std::stringstream os2;

    UtilLib::ProgressBar pb2(100000, 0.5, os);
    for (int i = 0; i < count; i++) {
        pb2++;
    }
    s = os2.str();
    BOOST_CHECK(s.find_first_of("progress: 0.002%") == std::string::npos);
}

void test_finished() {
    int count = 20;
    std::stringstream os;

    UtilLib::ProgressBar pb(count, 0.5, os);
    for (int i = 0; i < count; i++) {
        pb++;
    }
    std::string s = os.str();
    BOOST_CHECK(s.find_first_of(
                    "progress:  100%   estimated time remaining:     0 sec") ==
            0);
    BOOST_CHECK(s.find_first_of("overall runtime:") != std::string::npos);
}

int test_main(
        int argc,
        char* argv[]) { // note the name!
#ifdef ENABLE_MPI
    boost::mpi::environment env(argc, argv);

    // we use only two processors for this testing
    if (MPIProxy().getSize() != 2) {
        BOOST_FAIL("Run the test with two processes!");
    }
#endif
    // we use only two processors for this testing
    test_Constructor();
    test_timeUpdate();
    test_finished();

    return 0;
}
