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
#ifndef SOFA_COMPONENT_FORCEFIELD_FANFORCEFIELD_H
#define SOFA_COMPONENT_FORCEFIELD_FANFORCEFIELD_H

#include <MyAwesomeComponents/config.h>

#include <sofa/core/behavior/ForceField.h>
#include <sofa/helper/RandomGenerator.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/topology/BaseMeshTopology.h>


namespace sofa::component::forcefield
{

template<class DataTypes>
class FanForceField : public core::behavior::ForceField<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(FanForceField, DataTypes), SOFA_TEMPLATE(core::behavior::ForceField, DataTypes));

    typedef core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef core::objectmodel::Data<VecCoord> DataVecCoord;
    typedef core::objectmodel::Data<VecDeriv> DataVecDeriv;

public:
    /// Force applied at each point
    Data< Deriv > d_force;

protected:    
    /// Component constructor
    FanForceField();
    /// Pointer to the current topology
    sofa::core::topology::BaseMeshTopology* m_topology;
    
public:
    /// Init function
    void init() override;

    /// Forces addition for explicit and implicit integration schemes
    virtual void addForce (const core::MechanicalParams* params, DataVecDeriv& currentForce, const DataVecCoord& currentPosition, const DataVecDeriv& currentVelocities) override;

    /// Forces addition for implicit integration schemes
    virtual void addDForce(const core::MechanicalParams* /*mparams*/, DataVecDeriv& /*d_df*/ , const DataVecDeriv& /*d_dx*/) override {}

    virtual SReal getPotentialEnergy(const core::MechanicalParams* /*params*/, const DataVecCoord& /*x*/) const override { return 0; } // Keep it simple

public:

    /// Range for random force coefficient : [randForceMin ; randForceMax]
    Data<float> d_randForceMinCoeff;
    Data<float> d_randForceMaxCoeff;

    /// Probability to change random force coefficient
    Data<float> d_randForceCoeffChangeProba;

protected:
    /// Used to get random numbers
    sofa::helper::RandomGenerator m_randomGenerator;

    /// Random coefficient applied to forces
    float m_randForceCoeff;

};

} // namespace sofa::component::forcefield

#ifndef SOFA_COMPONENT_FORCEFIELD_FANFORCEFIELD_CPP
extern template class MYAWESOMECOMPONENTS_API FanForceField<defaulttype::Vec3Types>;
#endif

#endif // SOFA_COMPONENT_FORCEFIELD_FANFORCEFIELD_H
