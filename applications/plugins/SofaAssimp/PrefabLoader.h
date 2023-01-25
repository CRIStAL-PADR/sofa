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

#include <SofaAssimp/config.h>
#include <sofa/core/loader/SceneLoader.h>
#include <sofa/type/SVector.h>
#include <sofa/simulation/Node.h>
#include <sofa/component/constraint/projective/SkeletalMotionConstraint.h>

#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>           // Output data structure
#include <assimp/postprocess.h>     // Post processing flags

namespace sofaassimp
{

namespace
{
    using namespace sofa;
}

class SOFA_ASSIMP_API PrefabLoader : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(PrefabLoader,sofa::core::objectmodel::BaseObject);

    struct NodeInfo;

    // describing a link between Assimp Node and Sofa Node allowing us to build a node hierarchy
    struct NodeInfo
    {
        std::size_t             mChildIndex;		// index of the current child node to process
        aiNode*                 mAiNode;		// aiNode being processed
        sofa::simulation::Node::SPtr	mNode;			// corresponding Node created in the sofa scene graph
        NodeInfo*			mParentNode;		// parent node (useful to retrieve mesh skeleton and to compute world transformation matrix)
        aiMatrix4x4			mTransformation;	// matrix that transforms from node space to world space

        NodeInfo(aiNode* pAiNode, simulation::Node::SPtr pNode, NodeInfo* mParentNode = NULL) :
            mChildIndex(0),
            mAiNode(pAiNode),
            mNode(pNode),
            mParentNode(mParentNode),
            mTransformation()
        {
            if(mParentNode)
                mTransformation = mParentNode->mTransformation;

            if(pAiNode)
                mTransformation *= pAiNode->mTransformation;

            /*if(root)
            {
            	std::cout << pAiNode->mTransformation.a1 << " - " << pAiNode->mTransformation.b1 << " - " << pAiNode->mTransformation.c1 << " - " << pAiNode->mTransformation.d1 << std::endl;
            	std::cout << pAiNode->mTransformation.a2 << " - " << pAiNode->mTransformation.b2 << " - " << pAiNode->mTransformation.c2 << " - " << pAiNode->mTransformation.d2 << std::endl;
            	std::cout << pAiNode->mTransformation.a3 << " - " << pAiNode->mTransformation.b3 << " - " << pAiNode->mTransformation.c3 << " - " << pAiNode->mTransformation.d3 << std::endl;
            	std::cout << pAiNode->mTransformation.a4 << " - " << pAiNode->mTransformation.b4 << " - " << pAiNode->mTransformation.c4 << " - " << pAiNode->mTransformation.d4 << std::endl;
            }*/
        }

        NodeInfo(const NodeInfo& nodeInfo) :
            mChildIndex(nodeInfo.mChildIndex),
            mAiNode(nodeInfo.mAiNode),
            mNode(nodeInfo.mNode),
            mParentNode(nodeInfo.mParentNode),
            mTransformation(nodeInfo.mTransformation)
        {

        }
    };

    // describing a link between a Node and an Assimp Mesh
    struct MeshInfo
    {
        aiMesh*		mAiMesh;	// mesh being processed
        NodeInfo	mNodeInfo;		// its owner node

        MeshInfo(aiMesh* pAiMesh, NodeInfo pNodeInfo) :
            mAiMesh(pAiMesh),
            mNodeInfo(pNodeInfo)
        {

        }
    };

protected:
    PrefabLoader();
    ~PrefabLoader();
public:

    virtual void init() override;

	float getAnimationSpeed() const			{return animationSpeed.getValue();}
	void setAnimationSpeed(float speed)		{animationSpeed.setValue(speed);}

protected:

    bool readDAE (std::ifstream &file, const char* filename);

private:

    // build the joints and bones array used in the SkeletalMotionConstraint
    bool fillSkeletalInfo(const aiScene* scene, aiNode* meshParentNode, aiNode* meshNode, aiMatrix4x4 meshTransformation, aiMesh* mesh,
                          type::vector<component::constraint::projective::SkeletonJoint<defaulttype::Rigid3Types> >& skeletonJoints,
                          type::vector<component::constraint::projective::SkeletonBone>& skeletonBones) const;

    // clean the scene graph of its empty and useless intermediary nodes
    void removeEmptyNodes();

public:

    virtual std::string type() { return "The format of this scene is Collada (.dae)."; }

private:
    bool load();
    simulation::Node::SPtr subSceneRoot;		// the Node containing the whole Collada loaded scene

    Assimp::Importer importer;		// the Assimp importer used to easily load the Collada scene

    sofa::core::objectmodel::DataFileName d_filename;
	Data<float> animationSpeed; ///< animation speed
	Data<bool> generateCollisionModels; ///< generate point/line/triangle collision models for imported meshes
};

}
