/**
\page tutorial_02_cdesolver_page How to set up a Reaction-Diffusion Solver



\tableofcontents



<!--===========================================================================================-->
\section tutorial_02_cdesolver_R tutorial_02_CDESolver_R

For a general introduction to the reaction-advection-diffusion solvers, please refer to
[The Reaction-Diffusion Solver](\ref tutorial_01_cdesolver), here only the implementation
of the initial conditions and reaction terms are shown.

In an anonymous namespace, a few constants are defined.
These are standard parameters for the Schnakenberg Turing system (Please see [here](\ref modeldescription02)).
The time step depends on your discretization which you have to do by hand prior to setting up the simulation.
@code{.cpp}
namespace {
const double gamma = 100.0; ///< the gamma parameter
const double a = 0.1; ///< the a parameter
const double deltaT = 1.0e-4; ///< the time step
}
@endcode

\subsection initial_02_R The Initial Condition

For Turing patterns, it is beneficial to start with slightly random initial conditions.
Thereforce, we define a [random_device](<http://www.cplusplus.com/reference/random/random_device/>).
For all lattice sites which belong to a cell (so surrounding and interstitial fluid is excluded), we set a random
number with mean 1 and a small perturbation.
You'll notice that we divide by 5.0: again, this is Lattice-Boltzmann intrinsics (this depends on the stencil which is used).
If you are not familiar with the LB method, we suggest that you get an introduction elsewhere
(but note that you don't have understand this at this point).
@code{.cpp}
void tutorial_02_CDESolverD2Q5_L::initSolver() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-0.01, 0.01);

    if (this->physicalNode_->getDomainIdentifier() == 0) {
        for (auto d : cdeDirIter_) {
            distributions_[d] = 0.0;
        }
    }
    else {
        for (auto d : cdeDirIter_) {
            distributions_[d] = (1.0 + dis(gen))/5.0;
        }
    }
}
@endcode



\subsection reaction_02_R The Reaction for the Receptor

Now let's implement the reaction term for the receptor (Please see [here](\ref modeldescription02)):
First, we need to get the local concentrations of the receptor and the ligand.
The receptor is easy, because we're already inside that class.
The ligand concentration we'll have to get through the <tt>tutorial_02_CDESolverD2Q5_L</tt> solver:
@code{.cpp}
const double tutorial_02_CDESolverD2Q5_R::reaction() const
{
    const double R = this->getC();
    const double L = physicalNode_->getCDESolverSlow("tutorial_02_CDESolverD2Q5_L").getC();
@endcode

If we have a cell lattice site, we'll return the reaction term, otherwise zero:
@code{.cpp}
    if (this->physicalNode_->getDomainIdentifier() != 0) {
        return gamma * (a - R + R * R * L);
    }
    else {
        return 0.0;
    }
}
@endcode

<!--===========================================================================================-->
\section tutorial_02_cdesolver_L tutorial_02_CDESolver_L


For a general introduction to the reaction-advection-diffusion solvers, please refer to
[The Reaction-Diffusion Solver](\ref tutorial_01_cdesolver), here only the implementation
of the initial conditions and reaction terms are shown.

In an anonymous namespace, a few constants are defined.
These are standard parameters for the Schnakenberg Turing system (Please see [here](\ref modeldescription02)).
The time step depends on your discretization which you have to do by hand prior to setting up the simulation.
@code{.cpp}
namespace {
const double gamma = 100.0; ///< the gamma parameter
const double b = 0.9; ///< the b parameter
const double deltaT = 1.0e-4; ///< the time step
}
@endcode

\subsection initial_02_L The Initial Condition

For Turing patterns, it is beneficial to start with slightly random initial conditions.
Thereforce, we define a [random_device](<http://www.cplusplus.com/reference/random/random_device/>).
For all lattice sites which belong to a cell (so surrounding and interstitial fluid is excluded), we set a random
number with mean 1 and a small perturbation.
You'll notice that we divide by 5.0: again, this is Lattice-Boltzmann intrinsics (this depends on the stencil which is used).
If you are not familiar with the LB method, we suggest that you get an introduction elsewhere
(but note that you don't have understand this at this point).
@code{.cpp}
void tutorial_02_CDESolverD2Q5_L::initSolver() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<> dis(-0.01, 0.01);

    if (this->physicalNode_->getDomainIdentifier() == 0) {
        for (auto d : cdeDirIter_) {
            distributions_[d] = 0.0;
        }
    }
    else {
        for (auto d : cdeDirIter_) {
            distributions_[d] = (1.0 + dis(gen))/5.0;
        }
    }
}
@endcode


\subsection reaction_02_L The Reaction for the Ligand

Now let's implement the reaction term for the ligand (Please see [here](\ref modeldescription02)):
First, we need to get the local concentrations of the receptor and the ligand.
The ligand is easy, because we're already inside that class.
The receptor concentration we'll have to get through the <tt>tutorial_02_CDESolverD2Q5_R</tt> solver:
@code{.cpp}
const double tutorial_02_CDESolverD2Q5_L::reaction() const
{
    const double R = physicalNode_->getCDESolverSlow("tutorial_02_CDESolverD2Q5_R").getC();
    const double L = this->getC();
@endcode

If we are on a cell lattice site, we'll return the reaction term, otherwise zero.
Note that if you wish to have ligand degradation in the surrounding fluid, you could implement it easily.
@code{.cpp}
    if (this->physicalNode_->getDomainIdentifier() != 0) {
        return gamma * (b - R * R * L);
    }
    else {
        return 0.0;
    }
}
@endcode




*/
