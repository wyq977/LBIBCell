/**
\page installation Installation

\section build Build

<ol>
<li> Create an out-of-source-build directory:

> $ mkdir build

> $ cd build

<li> Run cmake:

> $ cmake [options] ../

You have the following options:
<dl>
<dt>
BUILD_TESTING [Default: TRUE]
</dt>
<dd>Compiles the tests.</dd>
<dt>
ENABLE_MEMORY_TEST [Default: FALSE]
</dt>
<dd>Compiles the memory tests. Note that this increases the duration of the test run significantly.</dd>
<dt>
DEBUGLEVEL [Default: logINFO]
</dt>
<dd>Sets the debug level. For available debug levels go to \ref provided_debug_levels.</dd>
<dt>
CMAKE_INSTALL_PREFIX [Default: /usr/local]
</dt>
<dd>Allows to specify the installation path.</dd>
<dt>
CMAKE_BUILD_TYPE
</dt>
<dd>Controls the type of build:
<ul><li>None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
</li><li>Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
</li><li>Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
</li><li>RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
</li><li>MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL)
</li></ul>
</dd>
</dl>

A frequently used example would be:

> $ cmake path_to_src  -DCMAKE_BUILD_TYPE=Debug
</li>


<li> To compile the project run:

> $ make


<li> To build the documentation, run:
> $ make doc

Open build/doc/html/index.html.

\section testing Testing
To run the included tests:
<ol>
<li> Call cmake with the BUILD_TESTING=TRUE option:
> $ cmake -DBUILD_TESTING=TRUE -DCMAKE_BUILD_TYPE=Debug path_to_src
</li>
<li> To execute the tests run:

> $ make test
</li>

</ol>








\section run Run
<ol>

<li>
To set the number of cores in a multi-core environment:
> $ export OMP_NUM_THREADS=num_of_threads
</li>

<li>

To run your-application, execute in build/apps/:
</li>
> $ ./your-application
</ol>


*/
