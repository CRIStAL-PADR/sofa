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
#include <sofa/helper/TypeInfo.h>
#include <vector>
#include <string>
namespace sofa::core::objectmodel
{

class SOFA_CORE_API AbstractClassInfo
{
private:
    const std::type_info* pt;

protected:
    AbstractClassInfo(const std::type_info* ti) { pt = ti; }
    virtual ~AbstractClassInfo(){}

public:
    /// The following was from BaseClass
    std::string compilationTarget;       ///< In which SOFA_TARGET is registered this type
    std::string namespaceName;           ///< The c++ namespace
    std::string typeName;                ///< The c++ typename
    std::string className;               ///< The 'sofa' object class name (can be customized)
    std::string templateName;            ///< The 'sofa' object's template name (can be customized)
    std::string shortName;
    std::vector<const AbstractClassInfo*> parents;

    sofa::helper::TypeInfo type() const { return sofa::helper::TypeInfo(*pt); }

    /// The following was from ClassInfo (to deprecate ?)
    const std::string& name() const { return className; }

    /// returns true iff c is a parent class of this
    bool hasParent(const AbstractClassInfo* c) const;

    /// returns true iff a parent class of this is named parentClassName
    bool hasParent(const std::string& parentClassName) const;

    virtual Base* dynamicCastToBase(Base* obj) const = 0;
    virtual void* dynamicCast(Base* obj) const = 0;
    virtual bool isInstance(Base* obj) const = 0;
};

} ///sofa::core::objectmodel
