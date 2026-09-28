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
#include <sofa/component/solidmechanics/spring/config.h>

#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/type/vector.h>
#include <sofa/helper/accessor.h>

namespace sofa::component::solidmechanics::spring
{

inline SReal defaultSpringStiffness = 100.f;
inline SReal defaultSpringDamping = 5.f;

/// Set of simple springs between particles
class AllowedIntervalSpringForceField : public sofa::core::behavior::ForceField<sofa::defaulttype::Vec1Types>
{
public:
    SOFA_CLASS(AllowedIntervalSpringForceField, SOFA_TEMPLATE(sofa::core::behavior::ForceField,DataTypes));

    typedef typename sofa::core::behavior::ForceField<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename Coord::value_type Real;
    typedef core::objectmodel::Data<VecDeriv>    DataVecDeriv;
    typedef core::objectmodel::Data<VecCoord>    DataVecCoord;

    typedef helper::ReadAccessor< Data< VecCoord > > RDataRefVecCoord;
    typedef helper::WriteAccessor< Data< VecCoord > > WDataRefVecCoord;
    typedef helper::ReadAccessor< Data< VecDeriv > > RDataRefVecDeriv;
    typedef helper::WriteAccessor< Data< VecDeriv > > WDataRefVecDeriv;

    typedef core::behavior::MechanicalState<DataTypes> MechanicalState;
    static constexpr auto N = DataTypes::spatial_dimensions;
    typedef type::Mat<N,N,Real> Mat;

    using Inherit::getPotentialEnergy;

    Data<SReal> d_ks;  ///< List of stiffness for the all springs. Must have the same size as indices1 & indices2, or if only one element, it will be applied to all springs. If empty, 0 will be applied everywhere
    Data<SReal> d_kd;  ///< List of damping for the all springs. Must have the same size as indices1 & indices2, or if only one element, it will be applied to all springs. If empty, 0 will be applied everywhere
    Data<type::vector<bool> > d_enabled; ///< List of boolean stating on the fact that the spring is enabled. Must have as same size than indices1 & indices2, or if only one element, it will be applied to all springs. If empty, True will be applied everywhere

    void init() override;

    /// Add the forces.
    void addForce(const core::MechanicalParams* mparams, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& v) override;
    void addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx) override;
    SReal getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord&  /* x */) const override;

protected:
    AllowedIntervalSpringForceField();
    virtual ~AllowedIntervalSpringForceField();
};

} // namespace sofa::component::solidmechanics::spring
