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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */
#include <UtilLib/config.hpp>
#include <UtilLib/include/MPIProxy.hpp>
#include <UtilLib/include/Exception.hpp>
#include <vector>

namespace UtilLib {
int MPIProxy_::_rank = 0;
int MPIProxy_::_size = 1;

MPIProxy_::MPIProxy_() {
#ifdef ENABLE_MPI
    mpi::communicator world;
    _rank = world.rank();
    _size = world.size();
#endif
}

MPIProxy_::~MPIProxy_() {}

int MPIProxy_::getRank() const {
    return _rank;
}

int MPIProxy_::getSize() const {
    return _size;
}

void MPIProxy_::barrier() {
#ifdef ENABLE_MPI
    mpi::communicator world;
    world.barrier();
#endif
}

void MPIProxy_::waitAll() {
#ifdef ENABLE_MPI
    LOG(utilities::logDEBUG4) << "wait all called with: " <<
    _mpiStatus.size() << " mpi statues";
    mpi::wait_all(_mpiStatus.begin(), _mpiStatus.end());

    _mpiStatus.clear();

#endif
}

#ifdef ENABLE_MPI
std::vector<boost::mpi::request> MPIProxy_::_mpiStatus;
#endif
} /* end namespace  */
