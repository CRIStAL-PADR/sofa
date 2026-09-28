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
#include <sofa/component/solidmechanics/spring/AllowedIntervalSpringForceField.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa::component::solidmechanics::spring
{

AllowedIntervalSpringForceField::AllowedIntervalSpringForceField()
{

}

AllowedIntervalSpringForceField::~AllowedIntervalSpringForceField()
{

}

void AllowedIntervalSpringForceField::init()
{
    Inherit::init();
}

void AllowedIntervalSpringForceField::addForce(const core::MechanicalParams* /* mparams */, DataVecDeriv& f_, const DataVecCoord& x_, const DataVecDeriv& /* v */)
{
    if(!mstate) {
        msg_info() << "No Mechanical State found, no force will be computed..." << "\n";
        return;
    }

    sofa::helper::WriteOnlyAccessor< DataVecDeriv > f = f_;
    sofa::helper::ReadAccessor< DataVecCoord > x = x_;
    f.resize(x.size());
    double offset=0.01;
    auto k = d_kd.getValue();
    for(sofa::Index i=0;i<x.size();i++)
    {
        auto value = x[i].x(); // we are 1D
        if(value < -offset)
            f[i] = -k * (value+offset);
        else if(value > +offset)
            f[i] = -k * (value-offset);
    }
}

void AllowedIntervalSpringForceField::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df_, const DataVecDeriv& dx_)
{
    if(!mstate) {
        msg_info() << "No Mechanical State found, no force will be computed..." << "\n";
        return;
    }

    sofa::helper::WriteAccessor< DataVecDeriv > df = df_;
    sofa::helper::ReadAccessor< DataVecDeriv > dx = dx_;
    sofa::helper::ReadAccessor< DataVecCoord > x = mstate->readPositions();

    Real kFactor = (Real)sofa::core::mechanicalparams::kFactorIncludingRayleighDamping(mparams, this->rayleighStiffness.getValue());
    auto k = d_kd.getValue();

    double offset=0.01;
    for(sofa::Index i=0;i<x.size();i++)
    {
        auto value = x[i].x();
        if(value <-offset || value >+offset)
            df[i] += sofa::type::Vec1d{k * dx[i].x()};
    }
}

SReal AllowedIntervalSpringForceField::getPotentialEnergy(const core::MechanicalParams* /*mparams*/, const DataVecCoord&  /* x */) const
{
    msg_error() << "getPotentialEnergy is not implemented in this component, please contact developper";
    return 0.0;
}

void registerAllowedIntervalSpringForceField(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(core::ObjectRegistrationData("A spring-based force field constraining a 1D value to be in specific intervals of value.")
                                 .add< AllowedIntervalSpringForceField >());
}

} // namespace sofa::component::solidmechanics::spring
