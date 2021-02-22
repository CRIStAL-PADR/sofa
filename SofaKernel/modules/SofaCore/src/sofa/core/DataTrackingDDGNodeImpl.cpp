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
#include <sofa/core/DataTrackingDDGNodeImpl.h>
#include <sofa/core/objectmodel/Base.h>

namespace sofa::core
{

void DataTrackerCallback::setCallback( std::function<sofa::core::objectmodel::ComponentState(const DataTracker&)> f)
{
    m_callback = f;
}

void DataTrackerCallback::update()
{
    updateAllInputsIfDirty();

    auto cs = m_callback(m_dataTracker);
    if (m_owner)
        m_owner->d_componentState.setValue(cs); // but what if the state of the component was invalid for a reason that doesn't depend on this update?
    cleanDirty();
}


void DataTrackerEngine::addCallback( std::function<sofa::core::objectmodel::ComponentState(void)> f)
{
    m_callbacks.push_back(f);
}

/// Each callback in the engine is called, setting its owner's component state to the value returned by the last callback.
/// Because each callback overwrites the state of the same component, it is important that within a component, all
/// callbacks perform the same checks to determine the value of the ComponentState.
void DataTrackerEngine::update()
{
    updateAllInputsIfDirty();
    core::objectmodel::ComponentState cs = core::objectmodel::ComponentState::Valid;

    for(auto& callback : m_callbacks)
        cs = callback();

    if (m_owner)
        m_owner->d_componentState.setValue(cs);
    cleanDirty();
}

}

