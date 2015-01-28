/**
\page tutorial_01_cdesolver_page How to set up a Reaction-Diffusion Solver

\tableofcontents
<!--===========================================================================================-->
\section tutorial_01_cdesolver The Reaction-Diffusion Solver

Implementing a reaction-diffusion solver from scratch requires a high level of understanding of the Lattice Boltzmann (LB) method.
However, luckily, you only have to adapt a few lines of code of the existing examples to adapt it to your needs.
So why didn't we hide all the LB implementation details from the user?
The reason is that you might wish to implement your own LB variant
(different stencils, thus different advection implementations, different collision operators).
In short, the current implementation is a trade-off between offering the full flexibility to the user, but still keeping it as
lean as possible.
Therefore, whenever you like to implement/add a new reaction-diffusion solver, we highly recommend to copy-paste an
existing solver (such as [tutorial_01_CDESolverD2Q5_SIGNAL](\ref LbmLib::solver::tutorial_01_CDESolverD2Q5_SIGNAL))
and adapt only a few member methods.
That's why we now only discuss those member functions which are subject to adaptation.
If you are already familiar with the LB method, you'll be able to figure out how the remaining member functions work using
the code documentation.


\subsection cdetoadapt Copy-Paste an Existing Reaction-Diffusion Solver: How?

First, it is necessary to choose a unique solver name. The name should ideally be descriptive.
In our example, we chose <tt>tutorial_01_CDESolverD2Q5_SIGNAL</tt> since it is for Tutorial 1, it uses the
D2Q5 LB stencil (you do not need to know what that means at this point), and the biochemical species is called
<tt>SIGNAL</tt>.

Let's start with the header file tutorial_01_CDESolverD2Q5_SIGNAL.hpp.
There is actually not much to do: simply exchange all the names of the old solver with your new solver name.
Please do not forget to also change the name of the include guard!

The corresponding source file tutorial_01_CDESolverD2Q5_SIGNAL.cpp requires - of course - much more adaptation.
Again, exchange all the names of the old solver with your new solver name.
Do not forget to adapt the header include:
@code{.cpp}
#include <LbmLib/include/solver/CDESolver/tutorial_01_CDESolverD2Q5_SIGNAL.hpp>
@endcode

and - very important - the name of the solver:
@code{.cpp}
const std::string tutorial_01_CDESolverD2Q5_SIGNAL::name = "tutorial_01_CDESolverD2Q5_SIGNAL";
@endcode

The implementation of the two crucial member methods ([initSolver](\ref LbmLib::solver::tutorial_01_CDESolverD2Q5_SIGNAL::initSolver) for the
initial condition and [reaction](\ref LbmLib::solver::tutorial_01_CDESolverD2Q5_SIGNAL::reaction) to implement the reaction term)
is discussed in the next sub-section \ref cdeimplementreaction.

\subsection cdeimplementreaction Implementing the Initial Condition and the Reaction Term

Let's first have a look at the constants (defined locally in an anonymous namespace for convenience;
you can also put them into the appropriate methods):
@code{.cpp}
namespace {
const double deltaT = 1.0;
const unsigned int SWITCHOFF_TIME = 5000;
const double SIGNAL_decay = 0.00001;
const double SIGNAL_production = 0.0001;
const double SIGNAL_initalcondition = 1.0;
}
@endcode
The constant <tt>deltaT</tt> is the LB time step; just leave it as it is.
Since we want to stop the production of <tt>SIGNAL</tt> after a while, we define <tt>SWITCHOFF_TIME</tt>,
as well as the production and decay rates, and the inital concentration.

<br/>
We want to initialize the solver as follows: inside the initial cell(s), the concentration
shall be uniformly 1; everywhere else (surrounding and interstitial fluid) it should be zero.
The initialization of the solver looks like this:
@code{.cpp}
void tutorial_01_CDESolverD2Q5_SIGNAL::initSolver() {
    if (this->physicalNode_->getDomainIdentifier() == 0) {
        for (auto d : cdeDirIter_) {
            distributions_[d] = 0.0;
        }
    }
    else {
        for (auto d : cdeDirIter_) {
            distributions_[d] = SIGNAL_initalcondition/5.0;
        }
    }
}
@endcode
Basically, from the current lattice site (<tt>physicalNode</tt>) we get the domain identifier.
If it is zero, we have surrounding or interstitial fluid, and the concentration is set to zero
(do you not have to know what 'distributions' are at this point).
For all the remaining domain identifiers (all the cells), it is set to <tt>SIGNAL_initalcondition/5.0</tt>.
Why divided by 5.0??
This depends on the LB stencil. Since we have 5 'distributions' (D2Q5 stencil), we divide the initial concentration
evenly amongst them.

<br/>
The reaction is implemented in a member function which returns the evaluation of the
reaction term at the current time step, for the current lattice site, and for the current domain identifier and
cell type of that lattice site.
In our example, we want to have a constant production of <tt>SIGNAL</tt> for
the initial cell (<tt>this->physicalNode_->getDomainIdentifier() == 1</tt>) and only for the initial phase of the
simulation (<tt>Parameters.getCurrentIteration()<SWITCHOFF_TIME</tt>), and a linear
degredation otherwise.

@code{.cpp}
const double tutorial_01_CDESolverD2Q5_SIGNAL::reaction() const
{
    if ((this->physicalNode_->getDomainIdentifier() == 1) && (Parameters.getCurrentIteration()<SWITCHOFF_TIME)) {
        return SIGNAL_production;
    }
    else {
        return -SIGNAL_decay*this->getC();
    }
}
@endcode

That is quite simpel, isn't it?



\subsection cdeaddtoproject Adding the Solver to the Project

We highly recommend to put the new header and source file into <tt>libs/LbmLib/include/solver/CDESolver</tt>
and <tt>libs/LbmLib/src/solver/CDESolver</tt>, respectively (but any other location will work as well).
Then there is only one step missing: letting cmake know that your new files are part of the project.
To do so, please add the following two lines to <tt>libs/LbmLib/CMakeLists.txt</tt>:

@code{none}
set(LBMLIB_SRCS
    ...
    src/solver/CDESolver/tutorial_01_CDESolverD2Q5_SIGNAL.cpp
    ...
)
@endcode

@code{none}
set(LBMLIB_HEADER
    ...
    include/solver/CDESolver/tutorial_01_CDESolverD2Q5_SIGNAL.hpp
    ...
)
@endcode

That's it. You dont even have to add the solvers in your main application file.
Why? Because your application already knows from the parameter input file (see \ref generalinput)
which reaction-diffusion solvers have to be executed.
*/
