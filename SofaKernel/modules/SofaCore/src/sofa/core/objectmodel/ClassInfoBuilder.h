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

#include <sofa/core/objectmodel/ClassInfo.h>
#include <sofa/core/objectmodel/ClassInfoId.h>
#include <sofa/core/objectmodel/ClassInfoBaseImpl.h>
#include <sofa/core/objectmodel/ClassInfoRepository.h>
#include <sofa/helper/NameDecoder.h>
#include <tuple>

using sofa::helper::NameDecoder;

namespace sofa::core::objectmodel
{

template<class T>
class ClassInfoBuilder;

template<class Tuple>
class Parents
{
public:
    constexpr static int nb() { return std::tuple_size<Tuple>::value; }

    static ClassInfo* get(const int i) { return getRec<nb()-1>(i); }

    template<int j>
    static ClassInfo* getRec(int i)
    {
        if(i==j)
            return ClassInfoBuilder<typename std::template tuple_element<j, Tuple>::type  >::get();
        if constexpr (j>0)
            return getRec<j-1>(i);
        return nullptr;
    }
};

template<>
class Parents<void>
{
    public:
     constexpr static int nb(){ return 0; }
     constexpr static ClassInfo* get(int){ return nullptr; }
};

template<class T>
class ClassInfoBuilder
{
public:
    static ClassInfo* get()
    {
        static ClassInfo* info = createNewInstance();
        return info;
    }

private:
    static ClassInfo* createNewInstance()
    {
        ClassInfo* info = new ClassInfoBaseImpl<T>();
        info->typeName = NameDecoder::getTypeName<T>();
        info->namespaceName = NameDecoder::getNamespaceName<T>();
        info->className = NameDecoder::getClassName<T>();
        info->templateName = NameDecoder::getTemplateName<T>();
        info->shortName = NameDecoder::getShortName<T>();

        info->parents.resize(Parents<typename T::ParentClasses>::nb());
        for(unsigned int i=0;i<info->parents.size();i++)
        {
            info->parents[i] = Parents<typename T::ParentClasses>::get(i);
        }
        sofa::core::objectmodel::ClassInfoRepository::Set(
                    sofa::core::objectmodel::ClassInfoId::GetClassId<T>(),
                    info, "");
        return info;
    }
};

} /// sofa::core::objectmodel
