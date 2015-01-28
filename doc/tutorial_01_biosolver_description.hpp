/**
\page tutorial_01_biosolver_description How to use the BioSolverXX

\tableofcontents

<!--===========================================================================================-->
\section emptybiosolver An Empty BioSolver

Here, we explain the basic structure of an empty BioSolver.
We recommend that, if you wish to implement a new custom BioSolver, that you
start from this empty BioSolver and make sure that it is properly working.
Once this is the case, the specific behaviour can be implemented.
In the next sections, the implementation of various biological behaviour is discussed.

\subsection emptybiosolversource The Source Code

Let us now have a look at BioSolverEmpty.hpp.
The header is surrounded by standard include guards:

@code{.cpp}
#ifndef BIOSOLVEREMPTY_HPP
#define BIOSOLVEREMPTY_HPP
@endcode

Next, the BioBaseSolver.hpp is included, together with a few standard headers:
@code{.cpp}
#include <LbmLib/include/solver/BioSolver/BioBaseSolver.hpp>
#include <string>
#include <vector>
@endcode

The BioSolvers are within the LbmLib::solver namespace:
@code{.cpp}
namespace LbmLib {
namespace solver {
@endcode

Now it is time to declare our class, which inherits from BioBaseSolver and recursively
uses itself as a template parameter:
@code{.cpp}
class BioSolverEmpty : public BioBaseSolver<BioSolverEmpty> {
@endcode

We have to implement one single virtual public member function.
This function is called at every timestep, and executes your custom
biological behaviour.
Since both a [GeometryHandler](\ref LbmLib::geometry::GeometryHandler) and a
[ForceSolver](\ref LbmLib::solver::ForceSolver) object are passed,
you have full access to the geometry, the fluid and CDE lattices, and the forces.
@code{.cpp}
 public:
    virtual void applyBioProcess(
            geometry::GeometryHandler& geometryhandler,
            solver::ForceSolver& forcesolver
            );
@endcode

Then we have a little more boilerplate code: the BioBaseSolver has to be a
friend class, the default constructor has to be private to prevent instantiation,
and our BioSolver needs to have a unique name.
Finally, the namespaces and include guards are closed.
@code{.cpp}
 private:
    friend class BioBaseSolver;
    BioSolverEmpty();
    static const std::string name;
};
}
}  // end namespace
#endif  // BIOSOLVEREMPTY_HPP
@endcode



Let us now discuss the implementation of our new class, which can be found in
BioSolverEmpty.cpp.
Be sides a few standard headers, we need to include our header, and the header to get access to the global
simulation paramters (such as domain size, current time step):
@code{.cpp}
#include <LbmLib/include/solver/BioSolver/BioSolverEmpty.hpp>
#include <LbmLib/include/GlobalSimulationParameters.hpp>
#include <vector>
#include <string>
#include <iostream>
@endcode

For convenience, we define our locally used simlation parameters in an
anonymous namespace (but this is a matter of taste).
We recommend to define the frequency of applying this BioSolver:
@code{.cpp}
namespace LbmLib {
namespace solver {
namespace {
    const unsigned int FREQUENCY = 1; ///< The frequency of applying this solver
}
@endcode


Our private default constructor is empty:
@code{.cpp}
BioSolverEmpty::BioSolverEmpty() : BioBaseSolver()
{}
@endcode

Here we are: the function implementation which executes your custom biological operations.
The first thing is to check whether it is time to execute the function (which
is defined by <tt>FREQUENCY</tt>); if not, return without having done anything.
In this case, the BioSolver simply prints a string to your terminal.
@code{.cpp}
void BioSolverEmpty::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    // this solver does nothing
    std::cout<<"I'm BioSolverEmpty"<<std::endl;

}
@endcode

Finally, we have to give an unique name. In your application,
you can add this BioSolver by simply knowing its name.
@code{.cpp}
const std::string BioSolverEmpty::name = "BioSolverEmpty";
}
}  // end namespace
@endcode


\subsection emptybiosolverinclude Including the BioSolverXX in the Project

Finally, we need to get it run.
This reqires only two simple steps.
First, you need to include the two files (the header and source file) in your
project.
To be consistent, we want to add the files into the directories
<tt>libs/LbmLib/include/solver/BioSolver/</tt> and <tt>libs/LbmLib/src/solver/BioSolver/</tt>
(but any other place works as well, of course).
Therefore, we have to add the following lines to the
<tt>libs/LbmLib/CMakeLists.txt</tt>:

@code{none}
set(LBMLIB_SRCS
    ...
    src/solver/BioSolver/BioSolverEmpty.cpp
    ...
)
@endcode

@code{none}
set(LBMLIB_HEADER
    ...
    include/solver/BioSolver/BioSolverEmpty.hpp
    ...
)
@endcode

The last step is to add your BioSolver to your application.
For instance, in tutorial_01.cpp, we would add the following line of code
(you'll figure out the right location when reading \ref tutorial_01):

@code{.cpp}
simRunner.addBioSolver("BioSolverEmpty");
@endcode

That's it, your new BioSolver works.
In the following sections, we will not discuss the boilerplate code anymore,
but only focus on the implementation of <tt>applyBioProcess(...)</tt>.

<!--===========================================================================================-->
\section tutorial_01_BioSolverAreaRegulator tutorial_01_BioSolverAreaRegulator
<!--===========================================================================================-->


For a general introduction on how to create a new BioSolver, please consult the section \ref emptybiosolver.
The implementation can be found in tutorial_01_BioSolverAreaRegulator.hpp and tutorial_01_BioSolverAreaRegulator.cpp.


First, we define some constants in an anonymous namespace which we will use later.
We want to execute this solver at every time step. The target area of regulated cells shall be 200 units.
The proportional constant <tt>KP</tt> defines how quickly the cell area is pulled towards the target area.
Finally, <tt>MAXSOURCE</tt> defines an upper threshold for the mass source.
@code{.cpp}
namespace {
    const unsigned int FREQUENCY = 1; ///< The frequency of applying this solver.
    const double TARGETAREA = 200.0; ///< The default target cell area.
    const double KP = 0.00001; ///< The proportional constant.
    const double MAXSOURCE = 0.004; ///< The threshold for the maximal source.
}
@endcode


Now the implementation of the function applyBioProcess(...) is discussed.
We specify two temporary maps. As keys, they store the domain identifiers of the cells, and the respective
cell area (or mass source) will be stored as the value.
@code{.cpp}
void tutorial_01_BioSolverAreaRegulator::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    std::map<unsigned int,double> areas;
    std::map<unsigned int,double> sourcemap;
    unsigned int domainid;

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }
@endcode


Now we compute the areas of the cell. This is actually implemented as a member function of
the [GeometryHandler](\ref LbmLib::geometry::GeometryHandler) object.
The result is stored in our temporary <tt>areas</tt> map:
@code{.cpp}
    areas = geometryhandler.computeAreas();
@endcode

Now, we populate the target area map. This map (which is a private member, please see
\ref tutorial_01_BioSolverAreaRegulator.hpp) also stores the domain identifier flag of the
cells as key (like the <tt>areas</tt> and <tt>sourcemap</tt> maps), and the target area as value.
Here, all the cells get the same target area <tt>TARGETAREA</tt>.
Cells of type 1 are later set to proliferate (this behaviour could already set here if desired).
@code{.cpp}
    for (auto it : areas) {
        if ( this->targetareamap_.find(it.first) == this->targetareamap_.end() ) { // not existing yet
            this->targetareamap_[it.first] = TARGETAREA; // default
        }
    }
@endcode

Now the sources needed to maintain the desried target area are computed.
The <tt>sourcemap</tt> tells us later what mass source a cell needs to converge towards its target area.
This is simply done by computing the difference between the target and actual area,
multiplied by fhe factor <tt>KP</tt>
(note that more sophisticated controllers, e.g. PID controllers, could be easily implemented here):
@code{.cpp}
    for (auto it : areas) {
        sourcemap[it.first] = KP*(this->targetareamap_[it.first] - areas[it.first]);
    }
@endcode

Now we want to modify our mass source map a little bit.
In particular, we do not want to exceed a certain maximal mass source (e.g. to limit the maximal growth rate).
Here, we allow for negative mass sources, so cells also have the ability to shrink.
@code{.cpp}
    // modify the sourcemap
    for (auto it : sourcemap)
    {
        if (sourcemap[it.first] > MAXSOURCE) { // upper threshold
            sourcemap[it.first] = MAXSOURCE;
        }
        else if (sourcemap[it.first] < -MAXSOURCE) {
            sourcemap[it.first] = -MAXSOURCE; // lower threshold
        }
    }
@endcode



Let us loop the entire computational domain now in order to set the previously
computed mass sources.
Since we can do this in parallel, we can use openMP for the outermost loop:
@code{.cpp}
#pragma omp parallel for schedule(dynamic) private(domainid)
    for (unsigned int ity = 0;
         ity < geometryhandler.getPhysicalNodes().size();
         ity++) { // loop y direction
        for (unsigned int itx = 0;
             itx < geometryhandler.getPhysicalNodes()[0].size();
             itx++) { // loop x direction
@endcode


For every lattice site, we now decide what mass source we apply.
If the domain identifier is zero (that means we have surrounding or interstitial fluid), we do not add anything.
For the cell type is 1, we add the maximal mass source <tt>MAXSOURCE</tt>, which leads to maximal
cell growth and thus maximal proliferation.
For the other cells, the area regulation kicks in: we use the previously computed mass sources.
@code{.cpp}
            domainid = geometryhandler.getPhysicalNodes()[ity][itx]->getDomainIdentifier();
            if (domainid != 0) {
                if (geometryhandler.getPhysicalNodes()[ity][itx]->getCellType()==1) { // max prolif for celltype=1
                    geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(MAXSOURCE);
                }
                else { // regulated area for other celltypes
                    geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(sourcemap[domainid]);
                }
            }
        }
    }
}
@endcode


<!--===========================================================================================-->
\section tutorial_01_BioSolverMembraneTension tutorial_01_BioSolverMembraneTension
<!--===========================================================================================-->

For a general introduction on how to create a new BioSolver, please consult the section \ref emptybiosolver.
Here, only the implementation of the function applyBioProcess(...) is discussed.
The implementation can be found in tutorial_01_BioSolverMembraneTension.hpp and tutorial_01_BioSolverMembraneTension.cpp.

Like already described above, we put our local constants into an anonymous namespace.
We only apply this BioSolver every 10th time step;
<tt>FORCE</tt> is the constant membrane force.
@code{.cpp}
namespace {
    const unsigned int FREQUENCY = 10; ///< The frequency of applying this solver
    const double FORCE = 0.01; ///< the surface tension force
    class Connection;
}
@endcode

Now let's discuss <tt>applyBioProcess(...)</tt> member method.
First, we get all the connections and store them in the <tt>tempconnectionmap</tt>:
@code{.cpp}
void tutorial_01_BioSolverMembraneTension::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    std::stringstream forcedescriptor;
    auto tempconnectionmap = geometryhandler.getGeometry().getConnections();

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }
@endcode

To renew all the membrane forces, we first have to remove the old ones.
The membrane forces have type 6 (please see <tt>apps/config/force.txt</tt> for a list of all available force types;
you can also add your own force types if you wish to do so), so we remove all type 6 forces:
@code{.cpp}
    forcesolver.deleteForceType(6);
@endcode

Finally, we loop all our connections and create new forces.
Note that we have to create pairs of forces, since one force only acts on one geometry node, so
we also have to create the mirrored force.
The forces are added by creating a stringstream with the corresponding attributes
(depending on the force type; please see <tt>apps/config/force.txt</tt>), which is
added to the forcesolver:
@code{.cpp}
#pragma omp parallel for schedule(static) private(forcedescriptor)
    for (size_t j=0;
         j<tempconnectionmap.size();
         ++j) {
        forcedescriptor.str( std::string() ); // reset
        forcedescriptor.clear(); //clear flags
        forcedescriptor << "6\t" // type
                        << tempconnectionmap[j]->getGeometryNodes().first->getId() << "\t" // nodeID 1
                        << tempconnectionmap[j]->getGeometryNodes().second->getId() << "\t" // nodeID 2
                        << FORCE;
        forcesolver.addForce(&forcedescriptor);

        forcedescriptor.str( std::string() ); // reset
        forcedescriptor.clear(); //clear flags
        forcedescriptor << "6\t" // type
                        << tempconnectionmap[j]->getGeometryNodes().second->getId() << "\t" // nodeID 2
                        << tempconnectionmap[j]->getGeometryNodes().first->getId() << "\t" // nodeID 1
                        << FORCE;
        forcesolver.addForce(&forcedescriptor);
    }
}
@endcode

<!--===========================================================================================-->
\section tutorial_01_BioSolverCellJunction tutorial_01_BioSolverCellJunction
<!--===========================================================================================-->

For a general introduction on how to create a new BioSolver, please consult the section \ref emptybiosolver.
Here, only the implementation of the function applyBioProcess(...) is discussed.
The implementation can be found in tutorial_01_BioSolverCellJunction.hpp and tutorial_01_BioSolverCellJunction.cpp.

As before, we put the local constants into an anonymous namespace.
The solver is executed every 10th iteration.
<tt>RADIUS</tt> is the radius within which a membrane point is looking for another memrbane point
of another cell to create a cell-cell junction.
If a cell-cell junction is formed, the spring constant will be <tt>K</tt> and the resting length <tt>L0</tt>.
@code{.cpp}
namespace {
    const unsigned int FREQUENCY = 10; ///< The frequency of applying this solver
    const double RADIUS = 1.0; ///< the cell junction radius
    const double K = 0.02; ///< the cell junction spring constant
    const double L0 = 0.5; ///< the cell junction resting length
    std::stringstream forcedescriptor; ///< string, as e.g. read from force.txt
}
@endcode


Similar to the \ref tutorial_01_BioSolverMembraneTension,
we have to remove all the cell-cell junctions before new ones are created.
Cell-cell junction forces are of type 7 (please see <tt>apps/config/force.txt</tt> for a list of all available force types;
you can also add your own force types if you wish to do so), so we remove all type 7 forces:
@code{.cpp}
void tutorial_01_BioSolverCellJunction::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {
    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }
    forcesolver.deleteForceType(7);
@endcode

The following lines of could could be simplified if you renounce parallel execution.
Since openMP cannot handle std::map (and other non-random access containeres) directly, we have to first copy the content (the geometry nodes)
into a std::vector:
@code{.cpp}
    auto tempnodesmap = geometryhandler.getGeometry().getGeometryNodes();
    auto nodes_length = tempnodesmap.size();
    std::vector<std::map<unsigned int,std::shared_ptr<nodes::GeometryNode> >::const_iterator> helper_nodes;
    auto nodes_it = tempnodesmap.begin();

    // create array of iterators sequentally:
    for (size_t j=0;
         j<nodes_length;
         ++j) {
        helper_nodes.push_back(nodes_it++);
    }
@endcode

Once this is done, we can loop all the geometry nodes in parallel:
@code{.cpp}
#pragma omp parallel for schedule(static) private(forcedescriptor)
    for (size_t j=0;
         j<nodes_length;
         ++j) {
@endcode

Now we come to a central line of code: for node <tt>j</tt>, we are looking for
other membrane nodes within <tt>RADIUS</tt>.
For that, we use the [getGeometryNodesWithinRadiusWithAvoidanceClosest](\ref LbmLib::geometry::Geometry::getGeometryNodesWithinRadiusWithAvoidanceClosest)
member method, which returns the pointer to the closest geometry node - or <tt>nullptr</tt> if there is none.
@code{.cpp}
        std::shared_ptr<nodes::GeometryNode> closestnode =
                geometryhandler.getGeometry().getGeometryNodesWithinRadiusWithAvoidanceClosest(helper_nodes[j]->second->getXPos(),
                                                                                               helper_nodes[j]->second->getYPos(),
                                                                                               RADIUS,
                                                                                               helper_nodes[j]->second->getDomainIdOfAdjacentConnections());
@endcode


If we have found a valid candidate, we gonna add a new junction.
As for the \ref tutorial_01_BioSolverMembraneTension, we actually have to add a pair of mirrored foreces,
since one force only acts on one geometry node.
The stringstream, which describes the force, is assembled according to the structure described in
<tt>apps/config/force.txt</tt>.
Finally, the new forces are added to the <tt>forcesolver</tt>.


@code{.cpp}
        if (closestnode != nullptr) {
            forcedescriptor.str( std::string() ); // reset
            forcedescriptor.clear(); //clear flags
            forcedescriptor << "7\t" // type
                            << helper_nodes[j]->second->getId() << "\t" // nodeID 1
                            << closestnode->getId() << "\t" // nodeID 2
                            << K << "\t" // spring constant
                            << L0; // resting length
            forcesolver.addForce(&forcedescriptor);

            forcedescriptor.str( std::string() ); // reset
            forcedescriptor.clear(); //clear flags
            forcedescriptor << "7\t" // type
                            << closestnode->getId() << "\t" // nodeID 1
                            << helper_nodes[j]->second->getId() << "\t" // nodeID 2
                            << K << "\t" // spring constant
                            << L0; // resting length
            forcesolver.addForce(&forcedescriptor);
        }
    }
}
@endcode


<!--===========================================================================================-->
\section tutorial_01_BioSolverDifferentiation tutorial_01_BioSolverDifferentiation
<!--===========================================================================================-->

For a general introduction on how to create a new BioSolver, please consult the section \ref emptybiosolver.
Here, only the implementation of the function applyBioProcess(...) is discussed.
The implementation can be found in tutorial_01_BioSolverDifferentiation.hpp and tutorial_01_BioSolverDifferentiation.cpp.


As always, we put the local constant in an anonymous namespace.
The <tt>THRESHOLD</tt> is a constant which is used for the differentiation.
To be precise, for every cell, we integrate the signaling modecule concentration over the cell areas.
Whenever this value will drop below that <tt>THRESHOLD</tt>, we will differentiate that cell from type 1 to type 2.

@code{.cpp}
namespace{
    const unsigned int FREQUENCY = 100; ///< The frequency of applying this solver
    const double THRESHOLD = 1.6; ///< threshold for differentiation policy
    std::once_flag flag; ///< helper flag
}
@endcode


In a first step, we compute the spatial integral of our signaling molecule over the cell area.
To do so, we use the method [computeAccumulatedDomainConcentrations](\ref LbmLib::geometry::GeometryHandler::computeAccumulatedDomainConcentrations).
It returns a map, where the keys are the domain identifiers of the cells, and the values the corresponding
integrated concentrations:
@code{.cpp}
void tutorial_01_BioSolverDifferentiation::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    std::map<unsigned int,double> accumulatedcellconcentrationmap =
    geometryhandler.computeAccumulatedDomainConcentrations("tutorial_01_CDESolverD2Q5_SIGNAL");
@endcode

Next, we create a map which takes the domain identifier as key, and a vector with all the connections of that cell.
By that, we achieve a sorting and convenient access of the connections according to their cellular affiliation.
@code{.cpp}
    this->cellDefinition_.clear();
    for (auto it : geometryhandler.getGeometry().getConnections()) {
        this->cellDefinition_[(*it).getDomainIdentifier()].push_back(it);
    }
@endcode

The following lines of code are only executed when the BioSolver is called the very first time.
It is basically the inital condition for the cell types: the fluid always defaults to 0, and every initially existing
cell defaults to cell type 1. Of course, this can be changed if you need something else.
Also note that, if you load your geometry from a vtk file, the CellTypeTrackerMap is already automatically initialized,
and the following lines of code are actually totally superfluous and will never be executed.
@code{.cpp}
    std::call_once(flag, [this,&geometryhandler](){
                   geometryhandler.getCellTypeTrackerMap().clear();
                   geometryhandler.getCellTypeTrackerMap()[0] = 0;  // fluid default
                   for (auto it : this->cellDefinition_) { // default all other cells to type 1
                       geometryhandler.getCellTypeTrackerMap()[it.first] = 1;
                   }
    });
@endcode

Now we can loop the map with the integrated concentrations, and decide whether we gonna differentiate that cell or not.
In this case, if the value drops below <tt>THRESHOLD</tt>, we will differentiate to cell type 2:
@code{.cpp}
    for(std::map<unsigned int,double>::iterator it = ++accumulatedcellconcentrationmap.begin(); //skip extracellular domain
        it != accumulatedcellconcentrationmap.end();
        it++) {
        if (it->second < THRESHOLD) {
            geometryhandler.getCellTypeTrackerMap()[it->first] = 2;
        }
    }
@endcode

The final step consists of copying the modified (differentiated) cell types to the lattice, i.e.
the cell type flag of each lattice site is updated accordingly:
@code{.cpp}
    geometryhandler.copyCellTypeToPhysicalNodes(
            geometryhandler.getCellTypeTrackerMap()
            );
}
@endcode


<!--===========================================================================================-->
\section tutorial_01_BioSolverCellDivision tutorial_01_BioSolverCellDivision
<!--===========================================================================================-->

For a general introduction on how to create a new BioSolver, please consult the section \ref emptybiosolver.
Here, only the implementation of the function applyBioProcess(...) is discussed.
The implementation can be found in tutorial_01_BioSolverCellDivision.hpp and tutorial_01_BioSolverCellDivision.cpp.


We set <tt>FREQUENCY</tt> to 100, i.e. the cells are only checked for cell division every 100th time step.
The maximal cell size is set to 380, which means that a cell exceeding this value will be divided.

@code{.cpp}
namespace {
    const unsigned int FREQUENCY = 100; ///< The frequency of applying this solver
    const double MAXCELLSIZE = 380; ///< The maximal are before division
}
@endcode



The first step is to update the internal map ([celldefinition_](<tt>BioSolverCellDivision::cellDefinition_</tt>)) which stores the domain identifier as key,
and all the connections of that cell as value:

@code{.cpp}
void tutorial_01_BioSolverCellDivision::applyBioProcess(geometry::GeometryHandler& geometryhandler,
                                            ForceSolver &forcesolver
                                            ) {
    std::map<unsigned int,double> areas;

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    this->updateCellDefinition(geometryhandler);
@endcode


Only when executed the first time, we have to initialize the local cell type map.
This map stores the domain identifiers of the cells as key, and the corresponding cell type flag as value
(initializing all the cells with type 1 is maybe not what you want in the general case, so you might adapt this
to your needs).
Also note that, if you load your geometry from a vtk file, the CellTypeTrackerMap is already automatically initialized,
and the following lines of code are actually totally superfluous and will never be executed.
@code{.cpp}
    if (geometryhandler.getCellTypeTrackerMap().size() == 0) {
        geometryhandler.getCellTypeTrackerMap().clear();
        geometryhandler.getCellTypeTrackerMap()[0] = 0;  // fluid default
        for (auto it : this->cellDefinition_) { // default all other cells to type 1
            geometryhandler.getCellTypeTrackerMap()[it.first] = 1;
        }
    }
@endcode


As previously for the \ref tutorial_01_BioSolverAreaRegulator, we now compute the areas of the cells.
The result is stored in a map, with the domain identifier of the cells as key, and the corresponding area as value.
Here, we overwrite the area for domain identifier zero (which is the surrounding fluid) in order to prevent
division of the surrounding fluid.
@code{.cpp}
    areas = geometryhandler.computeAreas();
    areas[0] = 0;
@endcode

We now loop all the cells, have a look at their area, and decide whether they are big enough
to be divided:
@code{.cpp}
    for (auto it = ++areas.begin(); // skip domainIdentifier=0
         it != areas.end();
         it++) {
        if (it->second > MAXCELLSIZE) {
@endcode

If yes, the cell is divided. The called function <tt>divideCell</tt> is relatively complex,
and thus written in a separate function (which will be discussed below):
@code{.cpp}
            this->divideCell(geometryhandler,it->first);
@endcode

Because <tt>divideCell</tt> just altered the connectivity (and introduced a new cell), we have to
update the local cell definition map (remember: domain identifier as key, all the corresponding connections as value):
@code{.cpp}
            this->updateCellDefinition(geometryhandler);
@endcode

We also have to add the new cell to the local cell type map
(remember: domain identifier as key, corresponding cell type as value).
Since the map is passed by reference, we can make changes here:
(checkout [getCellTypeTrackerMap](\ref LbmLib::geometry::GeometryHandler::getCellTypeTrackerMap)),
we can make changes
@code{.cpp}
            geometryhandler.getCellTypeTrackerMap()[this->cellDefinition_.rbegin()->first] =
                    geometryhandler.getCellTypeTrackerMap()[it->first];
@endcode

Now, the method [cureLattice](\ref LbmLib::geometry::GeometryHandler::cureLattice) guarantees that the
topological are consistent, i.e. the newly formed connections have to be integrated, connected to the lattice,
boundary conditions have to be set up, etc.
@code{.cpp}
            geometryhandler.cureLattice();
        }
    }
@endcode

Finally, the cell types (we just added a few new cells) have to be copied to the lattice
(because each lattice site has a cell type flag):
@code{.cpp}
    geometryhandler.copyCellTypeToPhysicalNodes(
                geometryhandler.getCellTypeTrackerMap()
                );
}
@endcode





So now, as promised, we discussed the member method <tt>divideCell</tt>.
A new domain identifier is generated (simply increased the highest value by one),
and the "affected" connections are stored in a temporary container.
The affected connections are those which intersect with the cell division line.
Ideally, only two affected connections are found; if less, something went really wrong; if more,
the cell might be concave, and the cell division might lead to 3 daughter cells (we also want to avoid that).
The cell division line itself can be defined in different ways. The software is shipped with two choices: random
direction, or perpendicular to the longest axis of the cell.
Here, we implemented the latter choice, discuss the corresponding method <tt>getTwoConnectionsLongestAxis</tt>
in the end of this section.
Of course, you can implement any other custom behaviour, depending on your biological hypothesis.
@code{.cpp}
void tutorial_01_BioSolverCellDivision::divideCell(geometry::GeometryHandler& geometryhandler,unsigned int domainidentifier)
{
    const unsigned int newCellID = this->cellDefinition_.rbegin()->first + 1;
    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > affectedConnections;

    affectedConnections = this->getTwoConnectionsLongestAxis(domainidentifier);
    //affectedConnections = this->getTwoConnectionsRandomDirection(domainidentifier);
@endcode

Now that we now which connections to cut, we detach the corresponding geometry nodes from those connections...
@code{.cpp}
    affectedConnections[0]->getGeometryNodes().first->setConnection<1>(nullptr);
    affectedConnections[0]->getGeometryNodes().second->setConnection<0>(nullptr);
    affectedConnections[1]->getGeometryNodes().first->setConnection<1>(nullptr);
    affectedConnections[1]->getGeometryNodes().second->setConnection<0>(nullptr);
@endcode

... and create two new connections which represent the cut.
The attributes (boundary conditions, cell type, etc. are inherited from the affected (old) connections).
@code{.cpp}
    const std::map<std::string, std::vector<std::string> > descriptor1 =
            affectedConnections[0]->getBoundaryConditionDescriptor();
    const std::map<std::string, std::vector<std::string> > descriptor2 =
            affectedConnections[1]->getBoundaryConditionDescriptor();
    if (descriptor1 != descriptor2) {
        lbm_fail("The method cannot handle cell division cases with heterogeneous boundary conditions.");
    }

    geometryhandler.createConnection(
                (*affectedConnections[0]).getGeometryNodes().first,
                (*affectedConnections[1]).getGeometryNodes().second,
                descriptor1,
                domainidentifier);

    geometryhandler.createConnection(
                (*affectedConnections[1]).getGeometryNodes().first,
                (*affectedConnections[0]).getGeometryNodes().second,
                descriptor1,
                domainidentifier);
@endcode


Now we can permanently remove the old connections:
@code{.cpp}
    for (auto it : affectedConnections) {
        geometryhandler.eraseConnection(it);
    }
@endcode


There is one more thing: the one part of the old cell, which became the new cell, has to be updated since
its connections still carry the old domain identifier flag.
So we just choose one cell and update the domain identifier to <tt>newCellID</tt>:
@code{.cpp}
    //step 4: bump domainID's of the connections
    std::shared_ptr<LbmLib::geometry::Connection>  CSTART = (*affectedConnections[1]).getGeometryNodes().second->getConnection<1>();
    std::shared_ptr<LbmLib::geometry::Connection>  C1 = CSTART;
    std::shared_ptr<LbmLib::geometry::Connection>  C2 = nullptr;
    CSTART->setDomainIdentifier(newCellID);
    while (C2 != CSTART) {
        C2 = (*C1).getGeometryNodes().second->getConnection<1>();
        C2->setDomainIdentifier(newCellID);
        C1 = C2;
    }

    LOG(UtilLib::logINFO) << "Cell with domainIdentifier=" <<
                             domainidentifier << " divided" <<
                             " at time " << Parameters.getCurrentIteration();
}
@endcode


So, how do we find the affected connections if we assume that the mother cell is cut
perpendicular to the longest axis?
This is implemented in the member method <tt>getTwoConnectionsLongestAxis</tt> which
we will discuss now.
Note that we make use of boost::geometry here; this is not really possible, but good to know
in case you want to use some more fancy functions of boost::geometry.
We just declare are few temporary variables which will be used later:
@code{.cpp}
const std::vector<std::shared_ptr<LbmLib::geometry::Connection> >
tutorial_01_BioSolverCellDivision::getTwoConnectionsLongestAxis(unsigned int domainidentifier)
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    std::vector<std::shared_ptr<LbmLib::geometry::Connection> > affectedconnections;
    double dist = 0.0;
    double tempdist = 0.0;
    double invslope = 0.0;
    double intercept = 0.0;
    std::shared_ptr<nodes::GeometryNode> N1 = nullptr;
    std::shared_ptr<nodes::GeometryNode> N2 = nullptr;
    point MIDPOINT;
    bool C1 = 0;
    bool C2 = 0;
@endcode

Now we loop all the connections of the cell which shall be divided,
and search for those two points which have the largest distance
(note that you could also conduct a PCA here in order to find principle axes):
@code{.cpp}
    for (auto it1 : this->cellDefinition_[domainidentifier]) {
        for (auto it2 : this->cellDefinition_[domainidentifier]) {
            tempdist = boost::geometry::distance(*((*it1).getGeometryNodes().first),
                                                 *((*it2).getGeometryNodes().first)
                                                 );
            if (tempdist > dist) {
                dist = tempdist;
                N1 = (*it1).getGeometryNodes().first;
                N2 = (*it2).getGeometryNodes().first;
            }
        }
    }
@endcode


Now that we found the longest axis, we compute the perpendicular line (the line which is used to cut the cells).
This line is assumed to go through the midpoint of the longest axis:
@code{.cpp}
    // step 2: compute perpendicular axis
    MIDPOINT.set<0>(0.5*(N1->getXPos()+N2->getXPos()));
    MIDPOINT.set<1>(0.5*(N1->getYPos()+N2->getYPos()));
    invslope = -(N2->getXPos()-N1->getXPos()) / (N2->getYPos()-N1->getYPos());
    intercept = MIDPOINT.get<1>() - invslope*MIDPOINT.get<0>();
    assert(std::isfinite(invslope));
    assert(std::isfinite(intercept));
@endcode

Finally, we can search for the intersections between the cut line, and the connections of the cell.
Basically, for each connection, we check whether both point of that connection are on the opposite side of the cut line.
So if point 1 is on one side, and point 2 on the other side, we found an intersection. Or the other way round.
If we haven't found extactly two connections, we'll throw an exception.
@code{.cpp}
    // step 3: find affected *Connection*s
    for (auto it : this->cellDefinition_[domainidentifier]) {
        C1 = (*it).getGeometryNodes().first->getYPos() > invslope*(*it).getGeometryNodes().first->getXPos() + intercept;
        C2 = (*it).getGeometryNodes().second->getYPos() < invslope*(*it).getGeometryNodes().second->getXPos() + intercept;
        if ( (C1 && C2) || (!C1 && !C2)) {
            affectedconnections.push_back(it);
        }
    }
    assert(affectedconnections.size()==2 && "More or less than 2 connections found in BioSolver::getTwoConnectionsLongestAxis().");
    return affectedconnections;
}
@endcode

Whew! That BioSolver is quite complex.
Luckily, if you want to implement other rules how to divide a cell, you can copy-past most of the code into your
new BioSolver. Most probably, you'll only have to adapt a few lines of code.

























<!--===========================================================================================-->
\section tutorial_02_BioSolverGrowth tutorial_02_BioSolverGrowth
<!--===========================================================================================-->


For a general introduction on how to create a new BioSolver, please consult the section \ref emptybiosolver.
The implementation can be found in tutorial_01_BioSolverAreaRegulator.hpp and tutorial_01_BioSolverAreaRegulator.cpp.


First, we define some constants in an anonymous namespace which we will use later.
We want to execute this solver at every time step.
The <tt>MAXSOURCE</tt> defines the mass source strength.
If the R^2L concentration locally exceeds the <tt>THRESHOLD</tt>, then <tt>MAXSOURCE</tt> is added.
@code{.cpp}
namespace {
    const unsigned int FREQUENCY = 1; ///< The frequency of applying this solver
    const double MAXSOURCE = 0.001; ///< The maximal mass increment which is added at each time step
    const double THRESHOLD = 1.0; ///< If R*R*L exceeds the threshold, mass is added
}
@endcode


Now the implementation of the function applyBioProcess(...) is discussed.
Nothing fancy. We immediately return if the solver is not applied at this time step.
@code{.cpp}
void tutorial_02_BioSolverGrowth::applyBioProcess(
        geometry::GeometryHandler& geometryhandler,
        solver::ForceSolver& forcesolver
        ) {

    if (Parameters.getCurrentIteration()%FREQUENCY != 0) {
        return;
    }

    double conc_R;
    double conc_L;
    double source;
@endcode





Let us loop the entire computational domain now.
Since we can do this in parallel, we can use openMP for the outermost loop:
@code{.cpp}
#pragma omp parallel for schedule(dynamic)
    for (unsigned int ity = 0;
         ity < geometryhandler.getPhysicalNodes().size();
         ity++) { // loop y direction
        for (unsigned int itx = 0;
             itx < geometryhandler.getPhysicalNodes()[0].size();
             itx++) { // loop x direction
@endcode



Since we only want to add mass for cells, we get rid of the surrounding and interstitial fluid.
For every lattice site, we now get the local concentration of <tt>tutorial_02_CDESolverD2Q5_R</tt>
and <tt>tutorial_02_CDESolverD2Q5_L</tt> and temporally store them.
@code{.cpp}
            if (geometryhandler.getPhysicalNodes()[ity][itx]->getDomainIdentifier() != 0 ) {

                conc_R = geometryhandler.getPhysicalNodes()[ity][itx]->getCDESolverSlow("tutorial_02_CDESolverD2Q5_R").getC();
                conc_L = geometryhandler.getPhysicalNodes()[ity][itx]->getCDESolverSlow("tutorial_02_CDESolverD2Q5_L").getC();
@endcode

If the complex concentration exceeds <tt>THRESHOLD</tt>, the source is set to be hight, and very small otherwise:
@code{.cpp}
                if (conc_R*conc_R*conc_L > THRESHOLD) {
                    source = MAXSOURCE;
                }
                else {
                    source = MAXSOURCE/10.0;
                }
@endcode

Finally, the source is assigned to the lattice site.
@code{.cpp}
                geometryhandler.getPhysicalNodes()[ity][itx]->getFluidSolver().addMass(source);
            }
        }
    }
}
@endcode


*/
