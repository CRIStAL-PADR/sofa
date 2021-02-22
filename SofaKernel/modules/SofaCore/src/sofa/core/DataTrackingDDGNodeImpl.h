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
#include <sofa/core/DataTrackingDDGNode.h>
#include <sofa/core/objectmodel/ComponentState.h>

#include <functional>

namespace sofa::core
{
    ///////////////////
    /// a DDGNode that automatically triggers its update function
    /// when asking for an output and any input changed.
    /// Similar behavior than a DataEngine, but this is NOT a component
    /// and can be used everywhere.
    ///
    /// Note that it contains a DataTracker (m_dataTracker)
    /// to be able to check precisly which input changed if needed.
    ///
    ///
    ///
    ///
    /// **** Implementation good rules: (similar to DataEngine)
    ///
    /// //init
    ///    addInput // indicate all inputs
    ///    addOutput // indicate all outputs
    ///    setDirtyValue(); // the engine must start dirty (of course, no output are up-to-date)
    ///
    ///  DataTrackerCallback is usually created using the "addUpdateCallback()" method from Base.
    ///  Thus the context is usually passed to the lambda making all public & private
    ///  attributes & methods of the component accessible within the callback function.
    ///  example:
    ///
    ///  addUpdateCallback("name", {&name}, [this](DataTracker& tracker){
    ///       // Increment the state counter but without changing the state.
    ///       return d_componentState.getValue();
    ///  }, {&d_componentState});
    ///
    ///  A member function with the same signature - core::objectmodel::ComponentState(DataTracker&) - can
    ///  also be used.
    ///
    ///  The update of the inputs is done for you before calling the callback,
    ///  and they are also cleaned for you after the call. Thus there's no need
    ///  to manually call updateAllInputsIfDirty() or cleanDirty() (see implementation of update()
    ///
    class SOFA_CORE_API DataTrackerCallback : public DataTrackingDDGNode
    {
    public:
        /// set the update function to call
        /// when asking for an output and any input changed.
        void setCallback(std::function<sofa::core::objectmodel::ComponentState(const DataTracker&)> f);

        /// Calls the callback when one of the data has changed.
        void update() override;

        inline void setOwner(sofa::core::objectmodel::Base* owner) { m_owner = owner; }

    protected:
        std::function<sofa::core::objectmodel::ComponentState(const DataTracker&)> m_callback;
        sofa::core::objectmodel::Base* m_owner {nullptr};
    };


    ///////////////////////
    class SOFA_CORE_API DataTrackerEngine : public DataTrackingDDGNode
    {
    public:
        [[deprecated("2020-06-17: DataTrackerEngine has been deprecated, use DataTrackerCallback instead. DataTrackerCallback only supports 1 callback at a time, but multiple DataTrackerCallbacks can be created within a single component")]]
        DataTrackerEngine() : DataTrackingDDGNode() {}
        /// set the update function to call
        /// when asking for an output and any input changed.
        void addCallback(std::function<sofa::core::objectmodel::ComponentState(void)> f);

        /// Calls the callback when one of the data has changed.
        void update() override;

    protected:
        std::vector<std::function<sofa::core::objectmodel::ComponentState(void)>> m_callbacks;
        std::string m_name {""};
        sofa::core::objectmodel::Base* m_owner {nullptr};
    };
    /////////////////////////



    /// A DDGNode that will call a given Functor as soon as one of its input changes
    /// (a pointer to this DataTrackerFunctor is passed as parameter in the functor)
    template <typename FunctorType>
    class DataTrackerFunctor : public core::objectmodel::DDGNode
    {
    public:

        DataTrackerFunctor( FunctorType& functor )
            : core::objectmodel::DDGNode()
            , m_functor( functor )
        {}

        /// The trick is here, this function is called as soon as the input data changes
        /// and can then trigger the callback
        void setDirtyValue() override
        {
            m_functor( this );

            // the input needs to be inform their output (including this DataTrackerFunctor)
            // are not dirty, to be sure they will call setDirtyValue when they are modified
            cleanDirtyOutputsOfInputs();
        }

        /// This method is needed by DDGNode
        void update() override{}

    private:
        DataTrackerFunctor(const DataTrackerFunctor&);
        void operator=(const DataTrackerFunctor&);
        FunctorType& m_functor; ///< the functor to call when the input data changed
    };

} // namespace sofa::core

