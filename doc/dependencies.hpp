/**
\page dependencies Dependencies

<ol>
<li>\ref compilers</li>
<li>\ref boost</li>
<li>\ref cmake</li>
<li>\ref doxygen</li>
<li>\ref vtk</li>
<li>\ref paraview</li>
</ol>

\section compilers Compiler
The software has been successfully compiled with gcc 4.7 or higher for -std=c++11 support. Check with:
> $ gcc -v
Follow the instructions on <http://gcc.gnu.org/> to download and install.

\section boost Boost
You need to have the boost libraries 1.54.0 or later installed on your computer.
Follow the instructions on <http://www.boost.org/> to download and install.

\section cmake CMake
cmake 2.8 or higher is required. Check with:
> $ cmake --version
Follow the instructions on <http://www.cmake.org/> to download and intstall.

\section doxygen Doxygen
doxygen 1.8 or later. check with:
> $ doxygen --version
follow the instructions on <http://www.stack.nl/~dimitri/doxygen/> to download and install.

\section vtk VTK

vtk version 5.8 or higher is required if the vtk reporters are used. follow the instructions on <http://http://www.vtk.org/>.

furthermore, the package libvtk5.8-qt might have to be installed if cmake reports the following warning:
> The imported target "vtkQtChart" references the file "/usr/lib/libvtkQtChart.so.5.8.0" [...]

\section paraview ParaView

this is not a dependency, but used to post-process and visualize the data. please visit <http://http://www.paraview.org/>


*/
