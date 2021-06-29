/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
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
#include <sofa/core/ObjectFactory.h>

#include "DisplacementField.h"
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/RGBAColor.h>
#include <sofa/gl/GLSLShader.h>

namespace sofaimplicitfield
{

using sofa::helper::getReadAccessor;
using sofa::type::RGBAColor;
using sofa::type::Mat3x3d;

/// Register in the Factory
int ImplicitFieldTransformClass = sofa::core::RegisterObject("registering of ImplicitFieldTransform class") .add<DisplacementField>();

DisplacementField::DisplacementField() :
    l_field(initLink("field", "The scalar field to displace")),
    l_topology(initLink("topology", "The mesh topology to use as interpolation field")),
    l_dofs(initLink("dofs", "The nodal values to interpolate."))
{
}

double DisplacementField::getValue(Vec3d &pos, int &domain)
{
    SOFA_UNUSED(domain);

    // Here are the tetrahedron's descriptions
    // l_topology->

    // Here are the moving position.
    // l_dofs->

    auto x = getReadAccessor(*l_dofs->read(sofa::core::VecCoordId::position()));

    Vec3d p0 = x[0] - pos;
    return l_field->getValue( p0 );
}

/// Debug rendering of the Displacement Field.
void DisplacementField::draw(const sofa::core::visual::VisualParams* v)
{

    auto drawtools = v->drawTool();
    auto x = getReadAccessor(*l_dofs->read(sofa::core::VecCoordId::position()));
    for(auto tetra : l_topology->getTetrahedra())
    {
        Vec3d r0 = x[tetra[0]];
        Vec3d r1 = x[tetra[1]];
        Vec3d r2 = x[tetra[2]];
        Vec3d r3 = x[tetra[3]];

        // Draws the deformed state.
        drawtools->drawTetrahedron(r0, r1, r2, r3, RGBAColor::red());

        // Draws the initial state
        Mat3x3d T = {r0-r3, r1-r3, r2-r3};
        T.transpose();
        Mat3x3d Tinv = T.inverted();

        Vec3d coef0 = Tinv * (r0-r3);
        Vec3d coef1 = Tinv * (r1-r3);
        Vec3d coef2 = Tinv * (r2-r3);

    }
}

} /// sofaimplicitfield


