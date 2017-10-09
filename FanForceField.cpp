/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2015 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#ifndef SOFA_COMPONENT_FORCEFIELD_FANFORCEFIELD_CPP
#define SOFA_COMPONENT_FORCEFIELD_FANFORCEFIELD_CPP

#include "FanForceField.inl"
#include <sofa/core/ObjectFactory.h>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

namespace sofa
{

namespace component
{

namespace forcefield
{

using namespace sofa::defaulttype;


SOFA_DECL_CLASS(FanForceField)

int FanForceFieldClass = core::RegisterObject("Random forces applied to all points")
#ifndef SOFA_FLOAT
        .add< FanForceField<Vec3dTypes> >(true)
#endif
#ifndef SOFA_DOUBLE
        .add< FanForceField<Vec3fTypes> >(true)
#endif
        ;

#ifndef SOFA_FLOAT
template class MYAWESOMECOMPONENTS_API FanForceField<Vec3dTypes>;
#endif
#ifndef SOFA_DOUBLE
template class MYAWESOMECOMPONENTS_API FanForceField<Vec3fTypes>;
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
