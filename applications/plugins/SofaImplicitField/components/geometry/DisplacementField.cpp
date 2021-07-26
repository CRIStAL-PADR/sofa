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

class DisplacementField::InternalData
{
public:
    GLuint vertexBufferObject;
    GLuint displacementBufferObject;
    GLuint barycentriCoordinatesBufferObject;

    std::vector<sofa::type::Vec3f> vertices;
    std::vector<sofa::type::Vec3f> displacements;
    std::vector<unsigned int> indices;
    bool isInited = false;
    GLuint mvpMatrixID ;

    void init()
    {
        std::cout << "DisplacementField init internal data" << std::endl;
        isInited = true;

        /// Create a vertex buffer object so we can draw it
        glGenBuffers(1, &vertexBufferObject);

        /// Create a vertex buffer object so we can draw it
        glGenBuffers(1, &displacementBufferObject);

        /// Create a vertex buffer object so we can draw it
        glGenBuffers(1, &barycentriCoordinatesBufferObject);
    }
};

DisplacementField::DisplacementField() :
    l_field(initLink("field", "The scalar field to displace")),
    l_topology(initLink("topology", "The mesh topology to use as interpolation field")),
    l_dofs(initLink("dofs", "The nodal values to interpolate.")),
    l_shader(initLink("shader", "The shader to use for the rendering."))
{
    data.reset(new InternalData());
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
    auto x = getReadAccessor(*l_dofs->read(sofa::core::VecCoordId::position()));
    auto x_0 = getReadAccessor(*l_dofs->read(sofa::core::VecCoordId::restPosition()));
    auto dt = v->drawTool();

    for(auto tetra : l_topology->getTetrahedra())
    {
        Vec3d r0 = x[tetra[0]];
        Vec3d r1 = x[tetra[1]];
        Vec3d r2 = x[tetra[2]];
        Vec3d r3 = x[tetra[3]];

        // Draws the initial state
        Mat3x3d T = {r0-r3, r1-r3, r2-r3};
        T.transpose();
        Mat3x3d Tinv = T.inverted();

        Vec3d coef0 = Tinv * (r0-r3);
        Vec3d coef1 = Tinv * (r1-r3);
        Vec3d coef2 = Tinv * (r2-r3);
    }

    if(!data.get())
        return;

    if(!data->isInited)
        data->init();

    data->vertices.clear();
    data->displacements.clear();
    for(sofa::Index i=0;i<x.size();++i)
    {
        auto& vertex = x[i];
        auto& vertex_at_rest_position = x_0[i];
        data->vertices.push_back(vertex);
        data->displacements.push_back( vertex_at_rest_position );
    }

    std::vector<unsigned int> indices;
    for(auto tetra : l_topology->getTetrahedra())
    {
        indices.push_back(tetra[0]);
        indices.push_back(tetra[1]);
        indices.push_back(tetra[2]);

        indices.push_back(tetra[1]);
        indices.push_back(tetra[0]);
        indices.push_back(tetra[3]);

        indices.push_back(tetra[0]);
        indices.push_back(tetra[2]);
        indices.push_back(tetra[3]);

        indices.push_back(tetra[2]);
        indices.push_back(tetra[1]);
        indices.push_back(tetra[3]);
    }

    ////// Compute sphere and depth
    double projMat[16];
    double modelMat[16];

    v->getProjectionMatrix(projMat);
    float fProjMat[16];
    for (unsigned int i = 0; i < 16; i++)
        fProjMat[i] = float(projMat[i]);

    v->getModelViewMatrix(modelMat);
    float fModelMat[16];
    for (unsigned int i = 0; i < 16; i++)
        fModelMat[i] = float(modelMat[i]);

    if(l_shader.get())
    {
        auto pm = l_shader->getUniform(l_shader->getCurrentIndex(), "projection_matrix");
        auto mm = l_shader->getUniform(l_shader->getCurrentIndex(), "object_matrix");
        auto uZNear = l_shader->getUniform(l_shader->getCurrentIndex(), "zNear");
        auto uZFar = l_shader->getUniform(l_shader->getCurrentIndex(), "zFar");

        l_shader->start();
        glUniform1f(uZNear, v->zNear());
        glUniform1f(uZFar, v->zFar());

        glUniformMatrix4fv(pm, 1, false, fProjMat);
        glUniformMatrix4fv(mm, 1, false, fModelMat);
    }else{
        msg_error() << "NO SHADER for rendering";
    }

    //glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LESS);

    //// Upload the geometry to the buffer.
    glBindBuffer(GL_ARRAY_BUFFER, data->vertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER,                                   // target, size in byte of the buffer, buffer adress,
                 sizeof(sofa::type::Vec3f)*data->vertices.size(),
                 data->vertices.data(), GL_DYNAMIC_DRAW);

    //// Upload the displacement data into the buffer.
    glBindBuffer(GL_ARRAY_BUFFER, data->displacementBufferObject);
    glBufferData(GL_ARRAY_BUFFER,
                 sizeof(sofa::type::Vec3f)*data->displacements.size(),
                 data->displacements.data(), GL_DYNAMIC_DRAW);

    /// Position
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, data->vertexBufferObject);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

    /// Displacement
    glEnableVertexAttribArray(1);                                   // activates the generic vertex array
    glBindBuffer(GL_ARRAY_BUFFER, data->displacementBufferObject);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);          // define an array of generic vertex attribute data

    /// Type, count, format, pointer
    glDrawElements(GL_TRIANGLES, 100, GL_UNSIGNED_INT, indices.data());
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    if(l_shader.get())
        l_shader->stop();

}

} /// sofaimplicitfield


