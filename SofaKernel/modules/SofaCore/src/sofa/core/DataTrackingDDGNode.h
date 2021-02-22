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
#include <sofa/core/DataTracker.h>
#include <sofa/core/objectmodel/DDGNode.h>

namespace sofa::core
{

/// A DDGNode with trackable input Data (containing a DataTracker)
class SOFA_CORE_API DataTrackingDDGNode : public core::objectmodel::DDGNode
{
public:
    DataTrackingDDGNode() : core::objectmodel::DDGNode() {}

    /// Create a DataCallback object associated with multiple Data fields.
    void addInputs(std::initializer_list<sofa::core::objectmodel::BaseData*> datas);
    void addOutputs(std::initializer_list<sofa::core::objectmodel::BaseData*> datas);

    /// Set dirty flag to false
    /// for the DDGNode and for all the tracked Data
    virtual void cleanDirty(const core::ExecParams* params = nullptr);

    /// utility function to ensure all inputs are up-to-date
    /// can be useful for particulary complex DDGNode
    /// with a lot input/output imbricated access
    void updateAllInputsIfDirty();

protected:

    /// @name Tracking Data mechanism
    /// each Data added to the DataTracker
    /// is tracked to be able to check if its value changed
    /// since their last clean, called by default
    /// in DataEngine::cleanDirty().
    /// @{

    DataTracker m_dataTracker;

    ///@}

private:
    DataTrackingDDGNode(const DataTrackingDDGNode&);
    void operator=(const DataTrackingDDGNode&);
};

} // namespace sofa::core

