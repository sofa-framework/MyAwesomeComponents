/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include <MyAwesomeComponents/config.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>


namespace myawesomecomponents
{

extern void registerFanForceField(sofa::core::ObjectFactory* factory);

extern "C" {

MYAWESOMECOMPONENTS_API
void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        // make sure that this plugin is registered into the PluginManager
        sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);
        
        first = false;
    }
}

MYAWESOMECOMPONENTS_API
const char* getModuleName()
{
    return MODULE_NAME;
}

MYAWESOMECOMPONENTS_API
const char* getModuleVersion()
{
    return MODULE_VERSION;
}

MYAWESOMECOMPONENTS_API
const char* getModuleLicense()
{
    return "LGPL";
}

MYAWESOMECOMPONENTS_API
const char* getModuleDescription()
{
    return "SOFA plugin example";
}

MYAWESOMECOMPONENTS_API
const char* getModuleComponentList()
{
    // string containing the names of the classes provided by the plugin
    return "FanForceField";
}

MYAWESOMECOMPONENTS_API
void registerObjects(sofa::core::ObjectFactory* factory)
{
    registerFanForceField(factory);
}


} // extern "C"

} // namespace myawesomecomponents
