==========================================
LBIBCELL
==========================================

=== REQUIREMENTS AND DEPENDENCIES ========
1)	the software has only been tested on linux operating systems such as Ubuntu, RedHat, CentOS

2)	gcc 4.7 or higher for -std=c++11 support. check with:
	$ gcc -v
	follow the instructions on http://gcc.gnu.org/ to download and install.

3) 	boost 1.54.0 or higher.
	follow the instructions on http://www.boost.org/ to download and install.

4) 	cmake 2.8 or higher. check with:
	$ cmake --version
	follow the instructions on http://www.cmake.org/ to download and intstall.

5)	doxygen 1.8 or later. check with:
	$ doxygen --version
	follow the instructions on http://www.stack.nl/~dimitri/doxygen/ to download and install.
	
6)	vtk 5.8 or higher.
	follow the instructions on http://www.vtk.org/


======= BUILD ============================
in lbibcell:
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ../
$ make


======= DOCUMENTATION ====================
in lbibcell/build:
$ make doc
open lbibcell/build/doc/html/index.html


======= TESTS ============================
in lbibcell/build:
$ make test


======= RUN ==============================
in lbibcell/build/apps:
$ ./your-application-name

to set the number of cores:
$ export OMP_NUM_THREADS=1


======= VISUALIZE RESULTS ================
the simulation results are written into lbibcell/build/apps/output/
when using the vtk format, the results can be visualized in ParaView (http://www.paraview.org/)
using matlab, the results can be visualized by running lbibcell/build/apps/postprocessing/visualize_all.m. please use the file in lbibcell/build/apps/postprocessing/, and not the one in lbibcell/apps/postprocessing/.
please consult the documentation for the output format if you use your own visualization tools


======= QUICK TUTORIAL ===================
1)	lbibcell/apps/config/tutorial_01_parameters.txt:
	set the number of iterations, data dump interval, domain size, relaxation parameter tau for the fluid and the reaction-advection-diffusion solvers(see in the supplementary material how to calculate)
	
2)	lbibcell/apps/config/tutorial_01_geometry.txt:
	contains the initial geometry, in this case a single round cell. the first section contains a list of geometry nodes:
	#Nodes (id	xPos	yPos)
	where the node id is a unique integer identifier, followed by the x and y coordinates.
	in the second part, the connections inbetween each two points are listed:
	#Connection (nodeId1	nodeId2	domainId	bsolver	cdesolver	...)
	whee nodeId1 and nodeId2 are the node identifiers of the nodes which shall be connected. domainId is an integer describing the identity of a cell. in this case, our single initial cell has a domainId=1. if the cell membranes shall act as impermeable no-flux boundary conditions, pairs of strings can be given, where 'bsolver' is the boundary solver, and 'cdesolver' is the affected reaction-advection-diffusion solver. in our case, no boundary conditions are applied to the cells.
	
3)	lbibcell/apps/config/force.txt:
	contains a list of forces. the format is explained in the header of force.txt. we start our simulation without initial forces, thus force.txt is empty.
	
4) 	lbibcell/apps/tutorial_01.cpp:
	the user-provided application file includes the library's header files, configures the simulation, sets up the BioSolvers and MassSolvers, configures the reporters (used to define how the data shall be dumped), and finally launches the simulation.
	here, the following solvers are used: MassSolverBoxOutlet (the boundary condition of the entire domain), BioSolverAreaRegulator (regulates the growth of the cells), BioSolverMembraneTension (adds membrane tension to the cell membranes), BioSolverCellJunction (creates and breakes cell-cell junctions to neighboring cells), BioSolverDifferentiation (decides when a cell has to differentiate), BioSolverCellDivision (decides when and how a cell will be divided).
	
	The given BioSolvers are just basic examples: the user is encouraged to implement and add custom biological processes, and may use the given examples as a guide.
	
5)	adding custom BioSolvers: a short guide
	i)	inherit from BioBaseSolver, similar to the given examples. don't forget to give a unique name to the new solver, by adding it as a string member.
	ii)	overload the virtual function applyBioProcess(geometry::GeometryHandler& geometryhandler, solver::ForceSolver& forcesolver). You can modify the geometry (by accessing geometryhandler) and the forces (by accessing forcesolver)
	iii)	add the new header and source file to lbibcell/libs/LbmLib/CMakeLists.txt
	iv)	now you can add the solver to your main application file. for example, you can add it to lbibcell/apps/tutorial_01.cpp by adding the line simRunner.addBioSolver("Your-new-BioSolver");
	
6)	except from the custom main simulation file (e.g. lbibcell/apps/tutorial_01.cpp) and the custom BioSolvers (which only have to be added to the cmake project), you do not need to modify the library's source code.
	the way to implement biologically motivated processes becomes clearer when studying the shipped BioSolvers in detail. for further information, please consult the documentation, or contact the authors.
	

	

	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
