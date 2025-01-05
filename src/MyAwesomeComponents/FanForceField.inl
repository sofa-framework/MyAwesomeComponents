/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
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
#pragma once

#include <MyAwesomeComponents/FanForceField.h>

namespace sofa::component::mechanicalload
{

template<class DataTypes>
FanForceField<DataTypes>::FanForceField()
    : d_force(initData(&d_force, "force", "applied force to all points"))
    , d_randForceMinCoeff(initData(&d_randForceMinCoeff, "randForceMinCoeff", ""))
    , d_randForceMaxCoeff(initData(&d_randForceMaxCoeff, "randForceMaxCoeff", ""))
    , d_randForceCoeffChangeProba(initData(&d_randForceCoeffChangeProba, "randForceCoeffChangeProba", ""))
{
    // Nothing more is done here
}


template<class DataTypes>
void FanForceField<DataTypes>::init()
{
    m_topology = this->getContext()->getMeshTopology(); // get the mesh topology to access to the points

    m_randomGenerator.initSeed( (unsigned int)time(NULL) ); // init random generator
    m_randForceCoeff = 1.0; // init random force coefficient

    Inherit::init(); // call parent class init
}


template<class DataTypes>
void FanForceField<DataTypes>::addForce(const core::MechanicalParams* /*params*/, DataVecDeriv& currentForce, const DataVecCoord& /*currentPosition*/, const DataVecDeriv& /*currentVelocities*/)
{
    float randProba = m_randomGenerator.random<float>(0, 1);
    if( randProba < d_randForceCoeffChangeProba.getValue() )
    {
        m_randForceCoeff = m_randomGenerator.random<float>(d_randForceMinCoeff.getValue(), d_randForceMaxCoeff.getValue()); // generating new random force coefficient
    }

    sofa::helper::WriteAccessor<core::objectmodel::Data< VecDeriv> > force = currentForce; // create writer on the current force
    for(sofa::Size i = 0 ; i < m_topology->getNbPoints() ; i++)
    {
        force[i] += d_force.getValue() * m_randForceCoeff; // Add asked force randomized with coeff
    }
}

} // namespace sofa::component::mechanicalload

