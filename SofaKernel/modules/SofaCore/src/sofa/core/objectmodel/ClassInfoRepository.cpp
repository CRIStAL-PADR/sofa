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
#include <typeindex>
#include <map>
#include <iostream>

#include <algorithm>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/NameDecoder.h>
#include "ClassInfo.h"
#include "ClassInfoId.h"
#include "ClassInfoRepository.h"

namespace sofa::core::objectmodel
{

class NameOnlyClassInfo : public ClassInfo
{
public:
    NameOnlyClassInfo(std::string className_, std::string typeName_) : ClassInfo(&typeid(NameOnlyClassInfo))
    {
        className = className_;
        typeName = typeName_;
    }

    Base* dynamicCastToBase(Base *) const override{return nullptr;}
    void* dynamicCast(Base *) const override{return nullptr;}
    bool isInstance(Base *) const override{return false;}
};

class NoClassInfo : public ClassInfo
{
public:
    NoClassInfo() : ClassInfo(&typeid(NoClassInfo))
    {
        className = "MissingClassInfo";
        typeName = "MissingClassInfo";
    }
    Base* dynamicCastToBase(Base *) const override{return nullptr;}
    void* dynamicCast(Base *) const override{return nullptr;}
    bool isInstance(Base *) const override{return false;}

    static ClassInfo* GetInstance(){ static NoClassInfo instance; return &instance; }
};


static std::vector<const ClassInfo*>& getStorage()
{
    static std::vector<const ClassInfo*> typeinfos {};
    return typeinfos;
}

std::vector<const ClassInfo*> ClassInfoRepository::GetRegisteredTypes(const std::string& target)
{
    bool selectAll = target == "";
    std::vector<const ClassInfo*> tmp;
    for(auto info : getStorage())
    {
        if(info==nullptr)
            continue;

        if(selectAll || info->compilationTarget == target)
            tmp.push_back(info);
    }
    return tmp;
}

const ClassInfo* ClassInfoRepository::Get(const ClassInfoId& tid)
{
    sofa::Size id = tid.id;
    auto& typeinfos = getStorage();

    if( id < typeinfos.size() && typeinfos[id] != nullptr)
        return typeinfos[id];

    msg_error("ClassInfoRegistry") << "Missing typeinfo for '"<< sofa::helper::NameDecoder::decodeFullName(tid.nfo)
                                   << "' (searching at index " << tid.id  << ")";

    return nullptr;
}

int ClassInfoRepository::AllocateNewTypeId(const std::type_info& nfo)
{
    auto& typeinfos = getStorage();
    //std::string name = sofa::helper::NameDecoder::decodeTypeName(nfo);
    //std::string typeName = sofa::helper::NameDecoder::decodeTypeName(nfo);
    //typeinfos.push_back(new NameOnlyClassInfo(name, typeName));
    typeinfos.push_back(NoClassInfo::GetInstance());
    return typeinfos.size()-1;
}


int ClassInfoRepository::Set(const ClassInfoId& tid, ClassInfo* info, const std::string &compilationTarget)
{
    if( info == nullptr )
        return -1;

    auto& typeinfos = getStorage();
    sofa::Index id = tid.id;

    //msg_info("ClassInfoRegistry") << " Trying to register '"<< info->className << "/" << tid.nfo.name() << "' at index " << id << "";

    info->compilationTarget = compilationTarget;
    if( id >= typeinfos.size() )
    {
        typeinfos.resize(id+1, NoClassInfo::GetInstance());
    }

    if( typeinfos[id] && typeinfos[id] != NoClassInfo::GetInstance())
    {
        if( typeinfos[id] != info)
        {
            msg_error("ClassInfoRegistry") << " Overriding a typeinfo "<< id << " from " << typeinfos[id]->className << " to " << info->className;
            info->compilationTarget = compilationTarget;
            typeinfos[id] = info;
            return 2;
        }
        return -1;
    }

    typeinfos[id] = info;
    return 1;
}


}
