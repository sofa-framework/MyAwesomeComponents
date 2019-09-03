/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, master				  *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ODESOLVER_EXPLICITBDFSOLVER_H
#define SOFA_COMPONENT_ODESOLVER_EXPLICITBDFSOLVER_H

#include <sofa/core/behavior/OdeSolver.h>

namespace sofa
{

namespace component
{

namespace odesolver
{

/** This solver is an explicit Backward Differentiation (BDF) time integrator.
    The formula of the scheme is given below:
    3/2*u(t+1) = 2*u(t) - 1/2*u(t-1) + dt * M^{-1} (Diff + M*Reac)
 */

class ExplicitBDFSolver : public sofa::core::behavior::OdeSolver
{
public:
    SOFA_CLASS(ExplicitBDFSolver, sofa::core::behavior::OdeSolver);
protected:
    ExplicitBDFSolver();
public:
    /// Solve function of the solver
    void solve(const core::ExecParams* params /* PARAMS FIRST */, double dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId /*vResult*/) override;

    /// Given an input derivative order (0 for position, 1 for velocity, 2 for acceleration),
    /// how much will it affect the output derivative of the given order.
    virtual double getIntegrationFactor(int inputDerivative, int outputDerivative) const override
    {
        const double dt = getContext()->getDt();
	    double matrix[3][3] = {
              { 1, dt, 0},
	      { 0, 1, dt},
              { 0, 0,  0}};
        if (inputDerivative >= 3 || outputDerivative >= 3)
            return 0;
        else
            return matrix[outputDerivative][inputDerivative];
    }

    /// Given a solution of the linear system,
    /// how much will it affect the output derivative of the given order.
    virtual double getSolutionIntegrationFactor(int outputDerivative) const override
    {
        const double dt = getContext()->getDt();
        double vect[3] = {0, dt, 1};
        if (outputDerivative >= 3)
            return 0;
        else
            return vect[outputDerivative];
    }

    /// Initialization function
    void init() override
    {
        OdeSolver::init();
        reinit();
        cpt = 0;
    }


    //@{
    /** Ids for the MultiVec containing the value saved from one time step to another */
    sofa::core::MultiVecCoordId previous1PosID;
    sofa::core::MultiVecCoordId previous2PosID;
    sofa::core::MultiVecCoordId tempID;
    sofa::core::MultiVecDerivId accFID;
    //@}


    /// Integer counting the iteration (particular case at t = 0)
    unsigned int cpt;

};

} // namespace odesolver

} // namespace component

} // namespace sofa

#endif //SOFA_COMPONENT_ODESOLVER_EXPLICITBDFSOLVER_H
