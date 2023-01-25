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
#include "PrefabLoader.h"
#include <sofa/simulation/Simulation.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/component/mass/UniformMass.h>
#include <sofa/component/topology/container/constant/MeshTopology.h>
#include <sofa/gl/component/rendering3d/OglModel.h>
#include <sofa/component/collision/geometry/PointModel.h>
#include <sofa/component/collision/geometry/LineModel.h>
#include <sofa/component/collision/geometry/TriangleModel.h>
#include <sofa/component/mapping/nonlinear/RigidMapping.h>
#include <sofa/component/mapping/linear/SkinningMapping.h>
#include <sofa/component/mapping/linear/BarycentricMapping.h>
#include <sofa/component/mapping/linear/IdentityMapping.h>
#include <sofa/component/constraint/projective/FixedConstraint.h>
#include <sofa/component/constraint/projective/SkeletalMotionConstraint.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/SetDirectory.h>
#include <stack>
#include <algorithm>


namespace sofaassimp
{

using namespace sofa;
using namespace sofa::type;
using namespace sofa::defaulttype;
using namespace sofa::core::loader;
using namespace sofa::component::statecontainer;
using namespace sofa::component::mass;
using namespace sofa::component::topology;
using namespace sofa::component::topology::container::constant;
using namespace sofa::gl::component::rendering3d;
using namespace sofa::component::mapping;
using namespace sofa::component::mapping::linear;
using namespace sofa::component::mapping::nonlinear;
using namespace sofa::component::collision;
using namespace sofa::component::collision::geometry;
using namespace sofa::component::constraint::projective;
using namespace sofa::simulation;

int PrefabLoaderClass = core::RegisterObject("Import a prefab from file (FBX or Gltf file format supported).")
        .add< PrefabLoader >();

PrefabLoader::PrefabLoader() :
    subSceneRoot()
  , importer()
  , d_filename(initData(&d_filename, "filename", "Path to the file to the filename to load the prefab from"))
  , animationSpeed(initData(&animationSpeed, 1.0f, "animationSpeed", "animation speed"))
  , generateCollisionModels(initData(&generateCollisionModels, true, "generateCollisionModels", "generate point/line/triangle collision models for imported meshes"))
{

}

PrefabLoader::~PrefabLoader()
{
    importer.FreeScene();
}

void PrefabLoader::init()
{
    load();
    if(0 == subSceneRoot)
        return;

    // retrieving parent node
    core::objectmodel::BaseContext* currentContext = getContext();
    Node* parentNode = dynamic_cast<Node*>(currentContext);
    if(!parentNode)
    {
        msg_error() << "No parent node";

        if(currentContext)
            msg_error() << "Context is : " << currentContext->getName();

        return;
    }

    subSceneRoot->setName(d_filename.getRelativePath());
    parentNode->addChild(subSceneRoot);

    // find how many siblings scene loaders there are upward the current one
    int sceneLoaderNum = 0;

    Node::ObjectIterator objectIt;
    for(objectIt = parentNode->object.begin(); objectIt != parentNode->object.end(); ++objectIt)
    {
        if(dynamic_cast<SceneLoader*>(objectIt->get()))
            ++sceneLoaderNum;

        if(this == *objectIt)
            break;
    }

    // place an iterator on the last scene loader generated node
    int sceneLoaderNodeNum = 0;

    Node::ChildIterator childIt = parentNode->child.begin();
    if(1 != sceneLoaderNum)
    {
        for(; childIt != parentNode->child.end() - 1; ++childIt)
        {
            ++sceneLoaderNodeNum;
            if(subSceneRoot == *childIt || sceneLoaderNum == sceneLoaderNodeNum)
                break;
        }
    }

    // swap our generated node position till it is at the right place
    for (Node::ChildIterator it = parentNode->child.end() - 1; it != childIt; --it)
        parentNode->child.swap(it, it - 1);
}

bool PrefabLoader::load()
{
    bool fileRead = false;

    // loading file
    const char* filename = d_filename.getFullPath().c_str();
    std::ifstream file(filename);

    if(!file.good())
    {
        msg_error() << "Error: PrefabLoader: Cannot read file '" << d_filename << "'.";
        return false;
    }

    // reading file
    fileRead = readDAE (file,filename);
    file.close();

    return fileRead;
}

const char* typeName(aiMetadataType type)
{
    switch(type)
    {
    case aiMetadataType::AI_AIMETADATA: return "METADATA";
    case aiMetadataType::AI_AISTRING: return "STRING";
    case aiMetadataType::AI_BOOL: return "BOOL";
    case aiMetadataType::AI_INT32: return "INT32";
    case aiMetadataType::AI_FLOAT: return "FLOAT";
    case aiMetadataType::AI_DOUBLE: return "DOUBLE";
    case aiMetadataType::AI_UINT64: return "INT64";
    case aiMetadataType::AI_AIVECTOR3D: return "VECTOR3";
    case aiMetadataType::FORCE_32BIT: return "FORCE_32BIT";
    }

    return "";
}

void addNode(const aiScene* scene, const aiNode* node, sofa::simulation::Node::SPtr sofa_node)
{
    auto meshNode = getSimulation()->createNewNode(node->mName.C_Str());
    sofa_node->addChild(meshNode);

    std::cout << "Node      : " << node->mName.C_Str() << std::endl;
    std::cout << "  children:" << node->mNumChildren << std::endl;
    std::cout << "      mesh:" << node->mNumMeshes << std::endl;
    if(node->mNumMeshes)
    {
        std::map<Vec3d,unsigned> vertexMap;

        auto currentAiMesh =  scene->mMeshes[node->mMeshes[0]];
        // generating a MeshTopology and filling up its properties
        MeshTopology::SPtr currentMeshTopology = sofa::core::objectmodel::New<MeshTopology>();
        {
            // adding the generated MeshTopology to its parent Node
            meshNode->addObject(currentMeshTopology);
            currentMeshTopology->setName(node->mName.C_Str());

            // filling up position array
            auto points = helper::getWriteOnlyAccessor(currentMeshTopology->seqPoints);
            points.resize(currentAiMesh->mNumVertices);
            for(unsigned int k = 0; k < currentAiMesh->mNumVertices; ++k)
            {
                auto aiVec3d = currentAiMesh->mVertices[k];
                points[k].set(aiVec3d.x, aiVec3d.y, aiVec3d.z);
            }

            unsigned int numTriangles = 0, numQuads = 0;
            for(unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                if( 3 == currentAiMesh->mFaces[k].mNumIndices )
                    ++numTriangles;
                else if( 4 == currentAiMesh->mFaces[k].mNumIndices )
                    ++numQuads;

            //type::vector<core::topology::BaseMeshTopology::Triangle>& seqTriangles = *currentMeshTopology->seqTriangles.beginEdit();
            auto triangles = helper::getWriteOnlyAccessor(currentMeshTopology->seqTriangles);
            triangles.reserve(numTriangles);

            auto quads = helper::getWriteOnlyAccessor(currentMeshTopology->seqQuads);
            quads.reserve(numQuads);

            for( unsigned int k = 0 ; k < currentAiMesh->mNumFaces ; ++k )
            {
                unsigned int *faceIndices = currentAiMesh->mFaces[k].mIndices;
                if(currentAiMesh->mFaces[k].mNumIndices==3)
                    triangles.push_back( sofa::core::topology::BaseMeshTopology::Triangle(faceIndices[0], faceIndices[1], faceIndices[2]));
                else if(currentAiMesh->mFaces[k].mNumIndices==4)
                    quads.push_back( sofa::core::topology::BaseMeshTopology::Quad(faceIndices[0], faceIndices[1],
                                                                                  faceIndices[2], faceIndices[3]));
            }
        }
    }

    if(node->mMetaData)
    {
        std::cout << " meta-data:" << node->mMetaData->mNumProperties << std::endl;
        for(size_t indice=0;indice<node->mMetaData->mNumProperties;indice++)
        {
            aiString tmp1;
            ai_int32 tmp2;

            node->mMetaData->Get(indice, tmp1);
            node->mMetaData->Get(indice, tmp2);

            std::cout << "      data '" << node->mMetaData->mKeys[indice].C_Str()
                      << "(" << typeName(node->mMetaData->mValues[indice].mType) << ")"
                      << "(" << tmp1.C_Str() << "," << tmp2 << ")" << std::endl;

        }
    }
    for(size_t indice=0;indice<node->mNumChildren;indice++)
    {
        addNode(scene, node->mChildren[indice], meshNode);
    }
}

bool PrefabLoader::readDAE (std::ifstream &/*file*/, const char* /*filename*/)
{
    // importing scene
    const aiScene* scene = importer.ReadFile(d_filename.getFullPath(), 0);

    if(!scene)
    {
        msg_error() << "Import failed : " << importer.GetErrorString();
        return false;
    }

    std::cout << "========== Scene " << scene->mName.C_Str() << "==============" <<  std::endl;
    std::cout << " animations:" << scene->mNumAnimations << std::endl;
    std::cout << "    cameras:" << scene->mNumCameras << std::endl;
    std::cout << "     lights:" << scene->mNumLights << std::endl;
    std::cout << "     meshes:" << scene->mNumMeshes << std::endl;
    std::cout << "  skeletons:" << scene->mNumSkeletons << std::endl;
    std::cout << "   textures:" << scene->mNumTextures << std::endl;
    std::cout << std::endl;

    // traversing the scene graph
    if(scene->mRootNode)
    {
        auto node = scene->mRootNode;

        auto sofa_node = getSimulation()->createNewNode("");
        subSceneRoot = sofa_node;

        std::cout << "  Node:" << node->mName.C_Str() << std::endl;
        std::cout << "  children:" << node->mNumChildren << std::endl;
        std::cout << "      mesh:" << node->mNumMeshes << std::endl;

        for(size_t indice=0;indice<node->mNumChildren;indice++)
        {
            addNode(scene, node->mChildren[indice], sofa_node);
        }
    }
    return true;
    {
        // use a stack to process the nodes of the scene graph in the right order, we link an assimp node with a Node with a NodeInfo
        std::stack<NodeInfo> nodes;

        subSceneRoot = getSimulation()->createNewNode("");
        nodes.push(NodeInfo(scene->mRootNode, subSceneRoot));

        int meshId = 0;

        // processing each node of the scene graph
        while(!nodes.empty())
        {
            // fast access node parent pointer
            NodeInfo& currentNodeInfo = nodes.top();
            NodeInfo* parentNodeInfo = currentNodeInfo.mParentNode;
            aiNode* currentAiNode = currentNodeInfo.mAiNode;
            Node::SPtr currentNode = currentNodeInfo.mNode;
            std::size_t& childIndex = currentNodeInfo.mChildIndex;
            aiMatrix4x4& currentTransformation = currentNodeInfo.mTransformation;

            std::cout << "INFO      :" << currentAiNode->mName.C_Str() << std::endl;
            std::cout << "  children:" << currentAiNode->mNumChildren << std::endl;
            std::cout << "      mesh:" << currentAiNode->mNumMeshes << std::endl;

            // process the node just one time
            if(0 == childIndex)
            {
                {
                    // if the aiNode contains a name do not change it because we will need it to retrieve the node when processing bones
                    std::stringstream nameStream(std::string(currentAiNode->mName.data, currentAiNode->mName.length));
                    if(nameStream.str().empty())
                        nameStream << childIndex++;
                    currentNode->setName(nameStream.str());

                    //std::cout << currentNode->getName() << std::endl;
                }

                // extract the node transformation to apply them later on its meshes
                aiVector3D aiScale, aiTranslation;
                aiQuaternion aiRotation;
                currentTransformation.Decompose(aiScale, aiRotation, aiTranslation);
                Quat<SReal> quat(aiRotation.x, aiRotation.y, aiRotation.z, aiRotation.w);

                Vec3d translation(aiTranslation.x, aiTranslation.y, aiTranslation.z);
                Vec3d rotation(quat.toEulerVector() / (M_PI / 180.0));
                Vec3d scale(aiScale.x, aiScale.y, aiScale.z);

                // useful to generate a unique index for each component of a node
                int componentIndex = 0;

                // for each mesh in the node
                for(unsigned int j = 0; j < currentAiNode->mNumMeshes; ++j, ++meshId)
                {

                    Node::SPtr meshNode = getSimulation()->createNewNode(currentAiNode->mName.C_Str());
                    currentNode->addChild(meshNode);

                    aiMesh* currentAiMesh = scene->mMeshes[currentAiNode->mMeshes[j]];

                    // generating a name
                    std::string meshName(currentAiMesh->mName.data, currentAiMesh->mName.length);

                    // the node representing a part of the current mesh construction (skinning, collision, visualization ...)
                    Node::SPtr currentSubNode = meshNode;

                    // generating a MechanicalObject and a SkinningMapping if the mesh contains bones and filling up theirs properties
                    MechanicalObject<Rigid3Types>::SPtr currentBoneMechanicalObject;
                    if(currentAiMesh->HasBones())
                    {
                        /*std::cout << "animation num : " << currentAiScene->mNumAnimations << std::endl;
                        std::cout << "animation duration : " << currentAiScene->mAnimations[0]->mDuration << std::endl;
                        std::cout << "animation ticks per second : " << currentAiScene->mAnimations[0]->mTicksPerSecond << std::endl;
                        std::cout << "animation channel num : " << currentAiScene->mAnimations[0]->mNumChannels << std::endl;*/

                        currentBoneMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Rigid3Types> >();
                        {
                            // adding the generated MechanicalObject to its parent Node
                            currentSubNode->addObject(currentBoneMechanicalObject);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentBoneMechanicalObject->setName(nameStream.str());

                            // filling up position coordinate array
                            currentBoneMechanicalObject->resize(currentAiMesh->mNumBones);

                            {
                                Data<Rigid3Types::VecCoord>* d_x = currentBoneMechanicalObject->write(core::VecCoordId::position());
                                Rigid3Types::VecCoord &x = *d_x->beginEdit();
                                for(unsigned int k = 0; k < currentAiMesh->mNumBones; ++k)
                                {
                                    aiMatrix4x4 offsetMatrix = currentAiMesh->mBones[k]->mOffsetMatrix;
                                    offsetMatrix.Inverse();

                                    // mesh space to world space
                                    offsetMatrix = currentTransformation * offsetMatrix;

                                    // extract the bone transformation
                                    aiVector3D aiBoneScale, aiBoneTranslation;
                                    aiQuaternion aiBoneRotation;
                                    offsetMatrix.Decompose(aiBoneScale, aiBoneRotation, aiBoneTranslation);

                                    Vec3d boneTranslation(aiBoneTranslation.x, aiBoneTranslation.y, aiBoneTranslation.z);
                                    Quat<SReal> boneQuat(aiBoneRotation.x, aiBoneRotation.y, aiBoneRotation.z, aiBoneRotation.w);

                                    x[k] = Rigid3Types::Coord(boneTranslation, boneQuat);
                                }
                                d_x->endEdit();
                            }
                        }

                        if(generateCollisionModels.getValue())
                        {
                            UniformMass<Rigid3Types>::SPtr currentUniformMass = sofa::core::objectmodel::New<UniformMass<Rigid3Types> >();
                            {
                                // adding the generated UniformMass to its parent Node
                                currentSubNode->addObject(currentUniformMass);

                                std::stringstream nameStream(meshName);
                                if(meshName.empty())
                                    nameStream << componentIndex++;
                                currentUniformMass->setName(nameStream.str());

                                currentUniformMass->setTotalMass(80.0);
                            }
                        }

                        // generating a SkeletalMotionConstraint and filling up its properties
                        SkeletalMotionConstraint<Rigid3Types>::SPtr currentSkeletalMotionConstraint = sofa::core::objectmodel::New<SkeletalMotionConstraint<Rigid3Types> >();
                        {
                            // adding the generated SkeletalMotionConstraint to its parent Node
                            currentSubNode->addObject(currentSkeletalMotionConstraint);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentSkeletalMotionConstraint->setName(nameStream.str());

                            currentSkeletalMotionConstraint->setAnimationSpeed(animationSpeed.getValue());

                            aiNode* parentAiNode = NULL;
                            if(parentNodeInfo)
                                parentAiNode = parentNodeInfo->mAiNode;

                            type::vector<SkeletonJoint<Rigid3Types> > skeletonJoints;
                            type::vector<SkeletonBone> skeletonBones;
                            fillSkeletalInfo(scene, parentAiNode, currentAiNode, currentTransformation, currentAiMesh, skeletonJoints, skeletonBones);
                            currentSkeletalMotionConstraint->setSkeletalMotion(skeletonJoints, skeletonBones);
                        }
                    }
                    else
                    {
                        currentBoneMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Rigid3Types> >();
                        {
                            // adding the generated MechanicalObject to its parent Node
                            currentSubNode->addObject(currentBoneMechanicalObject);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentBoneMechanicalObject->setName(nameStream.str());

                            // filling up position coordinate array
                            currentBoneMechanicalObject->resize(1);

                            {
                                Data<type::vector<Rigid3Types::Coord> >* d_x = currentBoneMechanicalObject->write(core::VecCoordId::position());
                                type::vector<Rigid3Types::Coord> &x = *d_x->beginEdit();

                                Vec3d boneTranslation(0.0, 0.0, 0.0);
                                Quat<SReal> boneQuat(0.0, 0.0, 1.0, 1.0);

                                x[0] = Rigid3Types::Coord(boneTranslation, boneQuat);

                                d_x->endEdit();
                            }
                        }

                        UniformMass<Rigid3Types>::SPtr currentUniformMass = sofa::core::objectmodel::New<UniformMass<Rigid3Types> >();
                        {
                            // adding the generated UniformMass to its parent Node
                            currentSubNode->addObject(currentUniformMass);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentUniformMass->setName(nameStream.str());
                        }

                        FixedConstraint<Rigid3Types>::SPtr currentFixedConstraint = sofa::core::objectmodel::New<FixedConstraint<Rigid3Types> >();
                        {
                            // adding the generated FixedConstraint to its parent Node
                            currentSubNode->addObject(currentFixedConstraint);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentFixedConstraint->setName(nameStream.str());

                            currentFixedConstraint->d_fixAll.setValue(true);
                        }
                    }

                    std::stringstream rigidNameStream;
                    if(currentAiMesh->HasBones())
                        rigidNameStream << "skinning_" << (int)meshId;
                    else
                        rigidNameStream << "rigid_" << (int)meshId;

                    Node::SPtr rigidNode = getSimulation()->createNewNode(rigidNameStream.str());
                    currentSubNode->addChild(rigidNode);

                    currentSubNode = rigidNode;

                    std::map<Vec3d,unsigned> vertexMap; // no to superimpose identical vertices

                    // generating a MechanicalObject and filling up its properties
                    MechanicalObject<Vec3Types>::SPtr currentMechanicalObject = sofa::core::objectmodel::New<MechanicalObject<Vec3Types> >();
                    {
                        // adding the generated MechanicalObject to its parent Node
                        currentSubNode->addObject(currentMechanicalObject);

                        std::stringstream nameStream(meshName);
                        if(meshName.empty())
                            nameStream << componentIndex++;
                        currentMechanicalObject->setName(nameStream.str());

                        currentMechanicalObject->setTranslation(translation.x(), translation.y(), translation.z());
                        currentMechanicalObject->setRotation(rotation.x(), rotation.y(), rotation.z());
                        currentMechanicalObject->setScale(scale.x(), scale.y(), scale.z());

                        // filling up position coordinate array
                        if(0 != currentAiMesh->mNumVertices)
                        {
                            int vertexIdx=0;
                            for(unsigned int k = 0; k < currentAiMesh->mNumVertices; ++k)
                            {
                                Vec3d v(currentAiMesh->mVertices[k][0], currentAiMesh->mVertices[k][1], currentAiMesh->mVertices[k][2]);
                                if( vertexMap.find(v) == vertexMap.end() ) vertexMap[v] = vertexIdx++;
                            }

                            currentMechanicalObject->resize(vertexMap.size());

                            {
                                Data<type::vector<Vec3Types::Coord> >* d_x = currentMechanicalObject->write(core::VecCoordId::position());
                                type::vector<Vec3Types::Coord> &x = *d_x->beginEdit();
                                for( std::map<Vec3d,unsigned>::iterator it=vertexMap.begin() , itend=vertexMap.end() ; it!=itend ; ++it )
                                    x[it->second] = it->first;

                                d_x->endEdit();
                            }
                        }
                    }

                    // generating a MeshTopology and filling up its properties
                    MeshTopology::SPtr currentMeshTopology = sofa::core::objectmodel::New<MeshTopology>();
                    {
                        // adding the generated MeshTopology to its parent Node
                        currentSubNode->addObject(currentMeshTopology);

                        std::stringstream nameStream(meshName);
                        if(meshName.empty())
                            nameStream << componentIndex++;
                        currentMeshTopology->setName(nameStream.str());

                        // filling up position array
                        currentMeshTopology->seqPoints.setParent(&currentMechanicalObject->x);

                        unsigned int numTriangles = 0, numQuads = 0;
                        for(unsigned int k = 0; k < currentAiMesh->mNumFaces; ++k)
                            if( 3 == currentAiMesh->mFaces[k].mNumIndices )
                                ++numTriangles;
                            else if( 4 == currentAiMesh->mFaces[k].mNumIndices )
                                ++numQuads;


                        type::vector<core::topology::BaseMeshTopology::Triangle>& seqTriangles = *currentMeshTopology->seqTriangles.beginEdit();
                        {
                            if( numTriangles ) seqTriangles.reserve(numTriangles);
                            type::vector<core::topology::BaseMeshTopology::Quad>& seqQuads = *currentMeshTopology->seqQuads.beginEdit();
                            if( numQuads ) seqQuads.reserve(numQuads);

                            for( unsigned int k = 0 ; k < currentAiMesh->mNumFaces ; ++k )
                            {
                                if( currentAiMesh->mFaces[k].mNumIndices==3 )
                                {
                                    unsigned int *faceIndices = currentAiMesh->mFaces[k].mIndices;

                                    const unsigned int &faceIndex0 = faceIndices[0];
                                    const unsigned int &faceIndex1 = faceIndices[1];
                                    const unsigned int &faceIndex2 = faceIndices[2];

                                    Vec3d v0(currentAiMesh->mVertices[faceIndex0][0], currentAiMesh->mVertices[faceIndex0][1], currentAiMesh->mVertices[faceIndex0][2]);
                                    Vec3d v1(currentAiMesh->mVertices[faceIndex1][0], currentAiMesh->mVertices[faceIndex1][1], currentAiMesh->mVertices[faceIndex1][2]);
                                    Vec3d v2(currentAiMesh->mVertices[faceIndex2][0], currentAiMesh->mVertices[faceIndex2][1], currentAiMesh->mVertices[faceIndex2][2]);

                                    seqTriangles.push_back( core::topology::BaseMeshTopology::Triangle( vertexMap[v0], vertexMap[v1], vertexMap[v2] ) );
                                }
                                else if( currentAiMesh->mFaces[k].mNumIndices==4 )
                                {
                                    unsigned int *faceIndices = currentAiMesh->mFaces[k].mIndices;

                                    const unsigned int &faceIndex0 = faceIndices[0];
                                    const unsigned int &faceIndex1 = faceIndices[1];
                                    const unsigned int &faceIndex2 = faceIndices[2];
                                    const unsigned int &faceIndex3 = faceIndices[3];

                                    Vec3d v0(currentAiMesh->mVertices[faceIndex0][0], currentAiMesh->mVertices[faceIndex0][1], currentAiMesh->mVertices[faceIndex0][2]);
                                    Vec3d v1(currentAiMesh->mVertices[faceIndex1][0], currentAiMesh->mVertices[faceIndex1][1], currentAiMesh->mVertices[faceIndex1][2]);
                                    Vec3d v2(currentAiMesh->mVertices[faceIndex2][0], currentAiMesh->mVertices[faceIndex2][1], currentAiMesh->mVertices[faceIndex2][2]);
                                    Vec3d v3(currentAiMesh->mVertices[faceIndex3][0], currentAiMesh->mVertices[faceIndex3][1], currentAiMesh->mVertices[faceIndex3][2]);

                                    seqQuads.push_back( core::topology::BaseMeshTopology::Quad( vertexMap[v0], vertexMap[v1], vertexMap[v2], vertexMap[v3] ) );
                                }
                            }

                            currentMeshTopology->seqQuads.endEdit();
                        }

                        currentMeshTopology->seqTriangles.endEdit();
                    }


                    if(generateCollisionModels.getValue())
                    {
                        TriangleCollisionModel<defaulttype::Vec3Types>::SPtr currentTriangleCollisionModel = sofa::core::objectmodel::New<TriangleCollisionModel<defaulttype::Vec3Types> >();
                        {
                            // adding the generated TriangleCollisionModel to its parent Node
                            currentSubNode->addObject(currentTriangleCollisionModel);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentTriangleCollisionModel->setName(nameStream.str());
                        }

                        LineCollisionModel<defaulttype::Vec3Types>::SPtr currentLineCollisionModel = sofa::core::objectmodel::New<LineCollisionModel<defaulttype::Vec3Types> >();
                        {
                            // adding the generated LineCollisionModel to its parent Node
                            currentSubNode->addObject(currentLineCollisionModel);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentLineCollisionModel->setName(nameStream.str());
                        }

                        PointCollisionModel<defaulttype::Vec3Types>::SPtr currentPointCollisionModel = sofa::core::objectmodel::New<PointCollisionModel<defaulttype::Vec3Types> >();
                        {
                            // adding the generated PointCollisionModel to its parent Node
                            currentSubNode->addObject(currentPointCollisionModel);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentPointCollisionModel->setName(nameStream.str());
                        }
                    }

                    if(currentAiMesh->HasBones())
                    {
                        {
                            SkinningMapping<Rigid3Types, Vec3Types>::SPtr currentSkinningMapping = sofa::core::objectmodel::New<SkinningMapping<Rigid3Types, Vec3Types> >();
                            {
                                // adding the generated SkinningMapping to its parent Node
                                currentSubNode->addObject(currentSkinningMapping);

                                std::stringstream nameStream(meshName);
                                if(meshName.empty())
                                    nameStream << componentIndex++;
                                currentSkinningMapping->setName(nameStream.str());

                                currentSkinningMapping->setModels(currentBoneMechanicalObject.get(), currentMechanicalObject.get());

                                type::vector<type::SVector<SkinningMapping<Rigid3Types, Vec3Types>::InReal> > weights;
                                type::vector<type::SVector<unsigned int> > indices;
                                type::vector<unsigned int> nbref;

                                indices.resize(vertexMap.size());
                                weights.resize(vertexMap.size());
                                nbref.resize(vertexMap.size(),0);

                                for(unsigned int k = 0; k < currentAiMesh->mNumBones; ++k)
                                {
                                    aiBone*& bone = currentAiMesh->mBones[k];

                                    for(unsigned int l = 0; l < bone->mNumWeights; ++l)
                                    {

                                        const unsigned int& vertexid = bone->mWeights[l].mVertexId;

                                        if(vertexid >= currentAiMesh->mNumVertices)
                                        {
                                            msg_info() << "Error: PrefabLoader::readDAE, a mesh could not be load : " << nameStream.str() << " - in node : " << currentNode->getName();
                                            return false;
                                        }

                                        Vec3d v(currentAiMesh->mVertices[vertexid][0], currentAiMesh->mVertices[vertexid][1], currentAiMesh->mVertices[vertexid][2]);

                                        unsigned int id = vertexMap[v];
                                        float weight = bone->mWeights[l].mWeight;

                                        if( std::find( indices[id].begin(), indices[id].end(), k ) == indices[id].end() )
                                        {
                                            weights[id].push_back(weight);
                                            indices[id].push_back(k);
                                            ++nbref[id];
                                        }

                                    }
                                }

                                currentSkinningMapping->setWeights(weights, indices, nbref);
                            }
                        }
                    }
                    else
                    {
                        RigidMapping<Rigid3Types, Vec3Types>::SPtr currentRigidMapping = sofa::core::objectmodel::New<RigidMapping<Rigid3Types, Vec3Types> >();
                        {
                            // adding the generated RigidMapping to its parent Node
                            currentSubNode->addObject(currentRigidMapping);

                            std::stringstream nameStream(meshName);
                            if(meshName.empty())
                                nameStream << componentIndex++;
                            currentRigidMapping->setName(nameStream.str());

                            currentRigidMapping->setModels(currentBoneMechanicalObject.get(), currentMechanicalObject.get());
                        }
                    }

                    // node used for visualization
                    std::stringstream visuNameStream;
                    visuNameStream << "visualization " << (int)meshId;

                    Node::SPtr visuNode = getSimulation()->createNewNode(visuNameStream.str());
                    currentSubNode->addChild(visuNode);

                    currentSubNode = visuNode;

                    // generating an OglModel and filling up its properties
                    OglModel::SPtr currentOglModel = sofa::core::objectmodel::New<OglModel>();
                    {
                        // adding the generated OglModel to its parent Node
                        currentSubNode->addObject(currentOglModel);

                        std::stringstream nameStream(meshName);
                        if(meshName.empty())
                            nameStream << componentIndex++;
                        currentOglModel->setName(nameStream.str());

                        if(0 != currentAiMesh->mNumVertices)
                        {
                            sofa::type::vector<OglModel::Deriv> normals;
                            normals.resize(currentAiMesh->mNumVertices);
                            memcpy(&normals[0], currentAiMesh->mNormals, currentAiMesh->mNumVertices * sizeof(aiVector3D));
                            currentOglModel->setVnormals(&normals);
                        }
                    }

                    IdentityMapping<Vec3Types, Vec3Types>::SPtr currentIdentityMapping = sofa::core::objectmodel::New<IdentityMapping<Vec3Types, Vec3Types> >();
                    {
                        // adding the generated IdentityMapping to its parent Node
                        currentSubNode->addObject(currentIdentityMapping);

                        std::stringstream nameStream(meshName);
                        if(meshName.empty())
                            nameStream << componentIndex++;
                        currentIdentityMapping->setName(nameStream.str());

                        currentIdentityMapping->setModels(currentMechanicalObject.get(), currentOglModel.get());
                    }
                }
            }

            // pop the current node when each one of its children have been processed
            if(childIndex == currentAiNode->mNumChildren)
            {
                nodes.pop();
            }
            // process next sub node
            else
            {
                // generating sub Node and filling up its properties
                // store it in the stack to process its children later
                NodeInfo subNodeInfo(currentAiNode->mChildren[childIndex], getSimulation()->createNewNode(""), &currentNodeInfo);
                nodes.push(subNodeInfo);

                // adding the generated node to its parent Node
                currentNode->addChild(subNodeInfo.mNode);

                // this child will be processed, go to the next one
                ++childIndex;
            }
        }
    }

    removeEmptyNodes();

    return true;
}

bool PrefabLoader::fillSkeletalInfo(const aiScene* scene, aiNode* meshParentNode, aiNode* meshNode, aiMatrix4x4 meshTransformation, aiMesh* mesh, type::vector<SkeletonJoint<Rigid3Types> > &skeletonJoints, type::vector<SkeletonBone>& skeletonBones) const
{
    // return now if their is no scene, no mesh or no skeletonBones
    if(!scene || !mesh || !mesh->HasBones())
    {
        msg_info() << "no mesh to load !";
        return false;
    }

    std::map<aiNode*, std::size_t> aiNodeToSkeletonJointIndex;

    // compute the mesh transformation into a rigid
    Mat4x4d meshWorldTranformation(meshTransformation[0]);
    Rigid3Types::Coord meshTransformationRigid;
    meshTransformationRigid.getCenter()[0] = meshWorldTranformation[0][3];
    meshTransformationRigid.getCenter()[1] = meshWorldTranformation[1][3];
    meshTransformationRigid.getCenter()[2] = meshWorldTranformation[2][3];
    Mat3x3d rot; rot = meshWorldTranformation;
    meshTransformationRigid.getOrientation().fromMatrix(rot);

    // register every SkeletonJoint
    for(unsigned int j = 0; j < scene->mNumAnimations; ++j)
    {
        // for now we just want to handle one animation
        if(1 == j)
            break;

        aiAnimation*& animation = scene->mAnimations[j];
        for(unsigned int k = 0; k < animation->mNumChannels; ++k)
        {
            aiNodeAnim*& channel = animation->mChannels[k];
            aiString& nodeName = channel->mNodeName;
            aiNode* node = scene->mRootNode->FindNode(nodeName);

            // create the corresponding SkeletonJoint if it does not exist
            std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            if(aiNodeToSkeletonJointIndex.end() == aiNodeToSkeletonJointIndexIterator)
            {
                skeletonJoints.push_back(SkeletonJoint<Rigid3Types>());
                aiNodeToSkeletonJointIndex[node] = skeletonJoints.size() - 1;
                aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            }
            else
            {
                return false;
            }
            SkeletonJoint<Rigid3Types>& skeletonJoint = skeletonJoints[aiNodeToSkeletonJointIndexIterator->second];

            aiVectorKey positionKey, scaleKey;
            aiQuatKey	rotationKey;

            unsigned int numKey = std::max(channel->mNumPositionKeys, channel->mNumRotationKeys);

            skeletonJoint.mTimes.resize(numKey);
            skeletonJoint.mChannels.resize(numKey);
            for(unsigned int l = 0; l < numKey; ++l)
            {
                SReal time = 0.0;
                aiMatrix4x4 transformation;

                if(l < channel->mNumPositionKeys)
                {
                    positionKey = channel->mPositionKeys[l];
                    time = positionKey.mTime;
                    aiMatrix4x4 position;
                    aiMatrix4x4::Translation(positionKey.mValue, position);
                    transformation = position;
                }

                if(l < channel->mNumRotationKeys)
                {
                    rotationKey = channel->mRotationKeys[l];
                    time = rotationKey.mTime;
                    aiMatrix4x4 rotation(rotationKey.mValue.GetMatrix());
                    transformation *= rotation;
                }

                Mat4x4d localTranformation(transformation[0]);

                Rigid3Types::Coord localRigid;
                localRigid.getCenter()[0] = localTranformation[0][3];
                localRigid.getCenter()[1] = localTranformation[1][3];
                localRigid.getCenter()[2] = localTranformation[2][3];
                Mat3x3d rot; rot = localTranformation;
                localRigid.getOrientation().fromMatrix(rot);

                skeletonJoint.mTimes[l] = time;
                skeletonJoint.mChannels[l] = localRigid;
            }
        }
    }

    // register every bone and link them to their SkeletonJoint (or create it if it has not been created)
    skeletonBones.resize(mesh->mNumBones);
    for(unsigned int i = 0; i < mesh->mNumBones; ++i)
    {
        aiBone*& bone = mesh->mBones[i];
        const aiString& boneName = bone->mName;

        // register the parents SkeletonJoint for each bone
        aiNode* node = scene->mRootNode->FindNode(boneName);

        // create the corresponding SkeletonJoint if it does not exist
        std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
        if(aiNodeToSkeletonJointIndex.end() == aiNodeToSkeletonJointIndexIterator)
        {
            skeletonJoints.push_back(SkeletonJoint<Rigid3Types>());
            aiNodeToSkeletonJointIndex[node] = skeletonJoints.size() - 1;
            aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
        }

        skeletonBones[i] = aiNodeToSkeletonJointIndexIterator->second;
    }

    // register every SkeletonJoint and their parents and fill up theirs properties
    for(std::size_t i = 0; i < skeletonJoints.size(); ++i)
    {
        aiNode*	node = NULL;

        // find the ai node corresponding to the SkeletonJoint
        for(std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.begin(); aiNodeToSkeletonJointIndexIterator != aiNodeToSkeletonJointIndex.end(); ++aiNodeToSkeletonJointIndexIterator)
        {
            if(i == aiNodeToSkeletonJointIndexIterator->second)
            {
                node = aiNodeToSkeletonJointIndexIterator->first;
                break;
            }
        }

        if(NULL == node)
            return false;

        std::size_t previousSkeletonJointIndex;
        bool firstIteration = true;

        // find parents node
        while(NULL != node)
        {
            // stop if we reach the mesh node or its parent
            if(meshNode == node || meshParentNode == node)
                break;

            // create the corresponding SkeletonJoint if it does not exist
            std::map<aiNode*, std::size_t>::iterator aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            if(aiNodeToSkeletonJointIndex.end() == aiNodeToSkeletonJointIndexIterator)
            {
                skeletonJoints.push_back(SkeletonJoint<Rigid3Types>());
                aiNodeToSkeletonJointIndex[node] = skeletonJoints.size() - 1;
                aiNodeToSkeletonJointIndexIterator = aiNodeToSkeletonJointIndex.find(node);
            }
            SkeletonJoint<Rigid3Types>& currentSkeletonJoint = skeletonJoints[aiNodeToSkeletonJointIndexIterator->second];

            // register the current node
            aiMatrix4x4 aiLocalTransformation = node->mTransformation;

            // compute the rigid corresponding to the SkeletonJoint
            Mat4x4d localTranformation(aiLocalTransformation[0]);

            Rigid3Types::Coord localRigid;
            localRigid.getCenter()[0] = localTranformation[0][3];
            localRigid.getCenter()[1] = localTranformation[1][3];
            localRigid.getCenter()[2] = localTranformation[2][3];
            Mat3x3d rot; rot = localTranformation;
            localRigid.getOrientation().fromMatrix(rot);

            // apply the mesh transformation to the skeleton root joint only
            // we know that this joint is the root if the corresponding aiNode is the mesh node or its parent
            aiNode* parentNode = node->mParent;
            if(meshNode == parentNode || meshParentNode == parentNode)
            {
                // compute the mesh transformation
                localRigid = meshTransformationRigid.mult(localRigid);

                // apply the mesh transformation to each channel if the skeleton root joint contains animation
                for(std::size_t kk = 0; kk < currentSkeletonJoint.mChannels.size(); ++kk)
                    currentSkeletonJoint.mChannels[kk] = meshTransformationRigid.mult(currentSkeletonJoint.mChannels[kk]);
            }

            currentSkeletonJoint.setRestPosition(localRigid);

            if(!firstIteration)
                skeletonJoints[previousSkeletonJointIndex].mParentIndex = aiNodeToSkeletonJointIndexIterator->second;

            firstIteration = false;
            previousSkeletonJointIndex = aiNodeToSkeletonJointIndexIterator->second;

            node = node->mParent;
        }
    }

    return true;
}

void PrefabLoader::removeEmptyNodes()
{
    // remove intermediary or empty nodes
    {
        std::stack<std::pair<Node::SPtr, std::size_t> > nodes;

        nodes.push(std::pair<Node::SPtr, std::size_t>(subSceneRoot, 0));
        while(!nodes.empty())
        {
            Node::SPtr& node = nodes.top().first;
            std::size_t& index = nodes.top().second;

            if(node->getChildren().size() <= index)
            {
                nodes.pop();

                if(nodes.empty())
                    break;

                Node::SPtr& parentNode = nodes.top().first;
                std::size_t& parentIndex = nodes.top().second;

                // remove the node if it has no objects
                if(node->object.empty())
                {
                    if(0 != node->getChildren().size())
                    {
                        // links its child nodes directly to its parent node before remove the current intermediary node
                        while(!node->getChildren().empty())
                        {
                            Node::SPtr childNode = static_cast<Node*>(node->getChildren()[0]);
                            parentNode->moveChild(childNode);
                        }
                    }

                    parentNode->removeChild(node);
                }
                else
                {
                    ++parentIndex;
                }
            }
            else
            {
                Node::SPtr child = static_cast<Node*>(node->getChildren()[index]);
                nodes.push(std::pair<Node::SPtr, std::size_t>(child, 0));
            }
        }
    }
}

}
