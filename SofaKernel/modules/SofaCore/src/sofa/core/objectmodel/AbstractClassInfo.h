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

#include <sofa/core/config.h>
#include <sofa/core/fwd.h>
#include <vector>
#include <string>
namespace sofa::core::objectmodel
{

class SOFA_CORE_API AbstractClassInfo
{

protected:
    AbstractClassInfo(){}
    virtual ~AbstractClassInfo(){}

public:
    std::string compilationTarget;
    std::string namespaceName;
    std::string typeName;
    std::string className;
    std::string templateName;
    std::string shortName;
    std::vector<const AbstractClassInfo*> parents;

    /// returns true iff c is a parent class of this
    bool hasParent(const AbstractClassInfo* c) const;

    /// returns true iff a parent class of this is named parentClassName
    bool hasParent(const std::string& parentClassName) const;

    virtual Base* dynamicCast(Base* obj) const = 0;
    virtual bool isInstance(Base* obj) const = 0;
};

} ///sofa::core::objectmodel
