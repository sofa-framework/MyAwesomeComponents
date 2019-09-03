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
#include <MyAwesomeComponents/ExplicitBDFSolver.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/MechanicalOperations.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/helper/Quater.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/behavior/MultiVec.h>
#include <math.h>
#include <iostream>


namespace sofa
{

namespace component
{

namespace odesolver
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace core::behavior;
using namespace core::objectmodel;

int ExplicitBDFSolverClass = core::RegisterObject("An explicit Backward Differentiation (BDF) time integrator")
        .add< ExplicitBDFSolver >()
	;

SOFA_DECL_CLASS(ExplicitBDFSolver);

ExplicitBDFSolver::ExplicitBDFSolver()
{
    cpt=0;
}

void ExplicitBDFSolver::solve(const core::ExecParams* params /* PARAMS FIRST */, double dt, sofa::core::MultiVecCoordId xResult, sofa::core::MultiVecDerivId /*vResult*/)
{
    // Vector and params definitions
    sofa::simulation::common::VectorOperations vop( params, this->getContext() );
    sofa::simulation::common::MechanicalOperations mop( params /* PARAMS FIRST */, this->getContext() );

    mop->setImplicit(false); // this solver is explicit only
    
    /// Get the state vectors (stored in the MechanicalState/Object)
    MultiVecCoord pos(&vop, core::VecCoordId::position() );
    MultiVecDeriv f  (&vop, core::VecDerivId::force() );
    MultiVecCoord newPos(&vop, xResult );


    // First Step : allocate memory for each new vector needed, through an ID
    if (cpt == 0 || this->getContext()->getTime()==0.0)
    {
        vop.v_alloc(accFID);
        vop.v_alloc(previous1PosID);
        vop.v_alloc(previous2PosID);
    }

	// From the ID, give a name to each MultiVec variable
    MultiVecDeriv accF(&vop, accFID);
    MultiVecCoord previous1Pos(&vop,  previous1PosID);
    MultiVecCoord previous2Pos(&vop,  previous2PosID);


    // the force contributions (right hand side of the equation)
    mop.computeForce(f);

    // Compute accF = forces f divided by the mass matrix
    mop.accFromF(accF, f);

	// Project and apply constraints on VEL
    mop.projectResponse(accF);
    mop.solveConstraint(accF, core::ConstraintParams::VEL);
    
	// Find solution
	pos.eq(pos, accF, dt);       
    
    // Apply the constraints on POS
    mop.solveConstraint(pos, core::ConstraintParams::POS);

    // Step incrementation
    cpt++;
}

} // namespace odesolver
} // namespace component
} // namespace sofa
