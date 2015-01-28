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

#ifndef UTILIB_MPIPROXY_HPP_
#define UTILIB_MPIPROXY_HPP_
#include <UtilLib/config.hpp>
#include <UtilLib/include/Log.hpp>
#include <UtilLib/include/Exception.hpp>
#include <UtilLib/include/Singleton.hpp>
#ifdef ENABLE_MPI
#include <boost/mpi/collectives.hpp>
#include <boost/mpi/communicator.hpp>
#include <boost/mpi/nonblocking.hpp>
#include <boost/mpi/request.hpp>
namespace mpi = boost::mpi;
#endif
#include <vector>


namespace UtilLib {
/**
 * @brief A class to handle all MPI related code. It also provides works if MPI is disabled
 *
 * This class encapsulate all MPI related code. The class also works if MPI is not enabled,
 * such that the code does not depend if MPI is enabled or not. At the moment only in the
 * main method the MPI environment needs to be generated. All other MPI calls are handled
 * by this class
 */
class MPIProxy_ {
 public:
    /**
     * destructor
     */
    virtual ~MPIProxy_();

    /**
     * wrapper method to return the process id, if mpi is disabled it returns 0
     * @return the world rank of a process
     */
    int getRank() const;

    /**
     * wrapper method to return the size, if MPI is disabled it returns 1
     * @return
     */
    int getSize() const;

    /**
     * wrapper for mpi barrier
     */
    void barrier();

    /**
     * waits until all request stored in the vector _mpiStatus are finished
     */
    void waitAll();

    /**
     * Broadcast the value from root
     * @param value The value to be broadcast
     * @param root The root process
     */
    template <typename T>
    void broadcast(
            T& value,
            int root);

    /**
     * asynchronous receive operation the mpi status is stored in _mpiStatus
     * @param source The source of the message
     * @param tag The tag of the message
     * @param value The value received
     */
    template <typename T>
    void irecv(
            int source,
            int tag,
            T& value) const;

    /**
     * asynchronous send operation the mpi status is stored in _mpiStatus
     * @param dest The destination of the message
     * @param tag The tag of the message
     * @param value The value sended
     */
    template <typename T>
    void isend(
            int dest,
            int tag,
            const T& value) const;

 private:
    /**
     * Declare the Singleton class a friend to allow construction of the MPIProxy_ class
     */
    friend class Singleton<MPIProxy_>;
    /**
     * constructor sets the MPI rank and size
     */
    MPIProxy_();

#ifdef ENABLE_MPI
    /**
     * stores the mpi statuses
     */
    static std::vector<boost::mpi::request> _mpiStatus;
#endif

    /**
     * storage of the rank to avoid function calls
     */
    static int _rank;

    /**
     * storage of the size to avoid function calls
     */
    static int _size;
};

template <typename T>
void MPIProxy_::broadcast(
        T& value,
        int root) {
#ifdef ENABLE_MPI
    mpi::communicator world;
    boost::mpi::broadcast(world, value, root);
#endif
}

template <typename T>
void MPIProxy_::irecv(
        int source,
        int tag,
        T& value) const {
#ifdef ENABLE_MPI
    mpi::communicator world;
    _mpiStatus.push_back(world.irecv(source, tag, value));
    LOG(utilities::logDEBUG4) << "recv source: " << source << "; tag: " <<
    tag << "; value: " << value;
#else
    UtilLib::Exception("MPI Code called from serial code in irecv");
#endif
}

template <typename T>
void MPIProxy_::isend(
        int dest,
        int tag,
        const T& value) const {
#ifdef ENABLE_MPI
    mpi::communicator world;
    _mpiStatus.push_back(world.isend(dest, tag, value));
    LOG(utilities::logDEBUG4) << "send destination: " << dest << "; tag: " <<
    tag << "; value: " << value;
#else
    UtilLib::Exception("MPI Code called from serial code in isend");
#endif
}

/**
 * Generate an singleton instance of the MPIProxy_ class.
 */
typedef Singleton<MPIProxy_> MPIProxySingleton;

/**
 * Wrapper function to reduce the writing needed to access the MPIProxy_.
 * inline this function to allow to definition in multiple translation units.
 * @return The reference to the single instance of MPIProxy
 */
inline MPIProxy_& MPIProxy() {
    return MPIProxySingleton::instance();
}
} /* end namespace */
#endif /* UTILIB_MPIPROXY_HPP_ */
