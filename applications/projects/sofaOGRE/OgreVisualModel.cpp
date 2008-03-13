#include "OgreVisualModel.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/io/Mesh.h>
#include <sofa/simulation/tree/GNode.h>

#include <iostream>

OgreVisualModel::OgreVisualModel()
: filename(dataField(&filename, "filename", "mesh file"))
, texturename(dataField(&texturename, "texturename", "texture image file"))
, materialname(dataField(&materialname, std::string("Examples/Chrome"), "materialname", "material name"))
, topology(NULL)
, nbNormals(0)
, ogreMesh(NULL), ogreEntity(NULL), ogreNode(NULL)
, useTexture(true), useNormals(true), modified(false)
{
}

std::string OgreVisualModel::getOgreName()
{
    return "SOFA"+static_cast<sofa::simulation::tree::GNode*>(getContext())->getPathName()+"/"+getName();
}

void OgreVisualModel::init()
{
    using sofa::helper::vector;
    using sofa::defaulttype::Vector3;
    
    ResizableExtVector< sofa::defaulttype::Vec2f > texcoords;
    if (filename.getValue() == "")
    {
        topology = dynamic_cast<sofa::component::topology::MeshTopology*>(getContext()->getTopology());
        if (topology != NULL)
        {
            int nbVOut = topology->getNbPoints();
            // Then we can create the final arrays
            x.resize(nbVOut);
            normals.resize(nbVOut);
            vertNormIdx.resize(nbVOut);
            for (int i = 0; i < nbVOut; i++)
            {
                x[i][0] = topology->getPX(i);
                x[i][1] = topology->getPY(i);
                x[i][2] = topology->getPZ(i);
                vertNormIdx[i] = i;
            }

            // Then we create the triangles
            int nbTIn = topology->getNbTriangles();
            int nbQIn = topology->getNbQuads();
            int nbTriangles = nbTIn + 2*nbQIn;
            triangles.resize(nbTriangles);
            int t = 0;
            for (int i = 0; i < nbTIn; i++)
            {
                sofa::component::topology::MeshTopology::Triangle e = topology->getTriangle(i);
                triangles[i][0] = e[0];
                triangles[i][1] = e[1];
                triangles[i][2] = e[2];
            }
            for (int i = 0; i < nbQIn; i++)
            {
                sofa::component::topology::MeshTopology::Quad e = topology->getQuad(i);
                triangles[nbTIn+2*i  ][0] = e[0];
                triangles[nbTIn+2*i  ][1] = e[1];
                triangles[nbTIn+2*i  ][2] = e[2];
                triangles[nbTIn+2*i+1][0] = e[0];
                triangles[nbTIn+2*i+1][1] = e[2];
                triangles[nbTIn+2*i+1][2] = e[3];
            }
        }
    }
    else
    {
        sofa::helper::io::Mesh *objLoader = NULL;
        objLoader = sofa::helper::io::Mesh::Create(filename.getValue());
        if (objLoader != NULL)
        {
            vector< vector< vector<int> > > &facetsImport = objLoader->getFacets();
            vector<Vector3> &verticesImport = objLoader->getVertices();
            vector<Vector3> &normalsImport = objLoader->getNormals();
            vector<Vector3> &texCoordsImport = objLoader->getTexCoords();
            
            std::cout << "Vertices Import size : " << verticesImport.size() << " (" << normalsImport.size() << " normals, " << texCoordsImport.size() << " texcoords)." << std::endl;
            
            int nbVIn = verticesImport.size();
            int nbTriangles = 0;
            // First we compute for each point how many pair of normal/texcoord indices are used
            // The map store the final index of each combinaison
            vector< std::map< std::pair<int,int>, int > > vertTexNormMap;
            vertTexNormMap.resize(nbVIn);
            for (unsigned int i = 0; i < facetsImport.size(); i++)
            {
                vector<vector <int> > vertNormTexIndex = facetsImport[i];
                if (vertNormTexIndex[0].size() < 3) continue; // ignore lines
                const vector<int>& verts = vertNormTexIndex[0];
                const vector<int>& texs = vertNormTexIndex[1];
                const vector<int>& norms = vertNormTexIndex[2];
                nbTriangles += verts.size()-2;
                for (unsigned int j = 0; j < verts.size(); j++)
                {
                    vertTexNormMap[verts[j]][std::make_pair((useTexture?texs[j]:-1), (useNormals?norms[j]:-1))] = 0;
                }
            }
            
            // Then we can compute how many vertices are created
            int nbVOut = 0;
            for (int i = 0; i < nbVIn; i++)
            {
                int s = vertTexNormMap[i].size();
                nbVOut += s;
            }
            
            // Then we can create the final arrays
            x.resize(nbVOut);
            normals.resize(nbVOut);
            vertNormIdx.resize(nbVOut);
            if (useTexture)
                texcoords.resize(nbVOut);
            
            nbNormals = 0;
            for (int i = 0, j = 0; i < nbVIn; i++)
            {
                std::map<int, int> normMap;
                for (std::map<std::pair<int, int>, int>::iterator it = vertTexNormMap[i].begin();
                     it != vertTexNormMap[i].end(); ++it)
                {
                    x[j] = verticesImport[i];
                    int t = it->first.first;
                    int n = it->first.second;
                    if ((unsigned)n < normalsImport.size())
                        normals[j] = normalsImport[n];
                    //n = -1;
                    if (useTexture && (unsigned)t < texCoordsImport.size())
                        texcoords[j] = texCoordsImport[t];
                    if (normMap.count(n))
                        vertNormIdx[j] = normMap[n];
                    else
                        vertNormIdx[j] = normMap[n] = nbNormals++;
                    it->second = j++;
                }
            }
            
            std::cout << "Vertices Export size : " << nbVOut << " (" << nbNormals << " normals)." << std::endl;
            std::cout << "Facets Import size : " << facetsImport.size() << std::endl;
            std::cout << "Facets Export size : " << nbTriangles << " triangles." << std::endl;
            
            // Then we create the triangles
            triangles.resize(nbTriangles);
            int t = 0;
            for (unsigned int i = 0; i < facetsImport.size(); i++)
            {
                vector<vector <int> > vertNormTexIndex = facetsImport[i];
                if (vertNormTexIndex[0].size() < 3) continue; // ignore lines
                vector<int> verts = vertNormTexIndex[0];
                vector<int> texs = vertNormTexIndex[1];
                vector<int> norms = vertNormTexIndex[2];
                vector<int> idxs;
                idxs.resize(verts.size());
                for (unsigned int j = 0; j < verts.size(); j++)
                    idxs[j] = vertTexNormMap[verts[j]][std::make_pair((useTexture?texs[j]:-1), (useNormals?norms[j]:-1))];
                
                for (unsigned int j = 2; j < verts.size(); j++)
                {
                    triangles[t][0] = idxs[0];
                    triangles[t][1] = idxs[j-1];
                    triangles[t][2] = idxs[j];
                    ++t;
                }
            }
        }
    }

    computeNormals();
    
    // Create OGRE Mesh
    ogreMesh = Ogre::MeshManager::getSingleton().createManual(getOgreName()+".mesh", "General"); //Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    Ogre::SubMesh *submesh = ogreMesh->createSubMesh();
    
    Ogre::VertexData* vertexData = ogreMesh->sharedVertexData = new Ogre::VertexData();
    Ogre::VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
    size_t currOffset[2] = {0, 0};
    // positions
    vertexDecl->addElement(0, currOffset[0], Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    currOffset[0] += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    // normals
    vertexDecl->addElement(0, currOffset[0], Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    currOffset[0] += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    if (useTexture)
    {
        // two dimensional texture coordinates
        vertexDecl->addElement(1, currOffset[1], Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
        currOffset[1] += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
    }
    // allocate the vertex buffer
    vertexData->vertexCount = x.size();
    vBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), (vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, false);
    Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
    binding->setBinding(0, vBuffer);
    
    uploadVertices();
    
    // Upload tex coordinates
    if (useTexture)
    {
        Ogre::HardwareVertexBufferSharedPtr vBuffer2 = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(1), (vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
        Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
        binding->setBinding(1, vBuffer2);
        vBuffer2->writeData(0, texcoords.size() * sizeof(texcoords[0]), texcoords.ptr());
    }
    
    submesh->useSharedVertices = true;
    // allocate index buffer
    submesh->indexData->indexCount = 3 * getNbTriangles();
    iBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer((x.size() < 0x10000) ? Ogre::HardwareIndexBuffer::IT_16BIT : Ogre::HardwareIndexBuffer::IT_32BIT, (submesh->indexData->indexCount>0?submesh->indexData->indexCount:1), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
    submesh->indexData->indexBuffer = iBuffer;
    
    uploadIndices();
    
    computeBounds();
    
    submesh->setMaterialName(materialname.getValue());
    
    // this line makes clear the mesh is loaded (avoids memory leaks)
    ogreMesh->load();
}

void OgreVisualModel::attach(Ogre::SceneManager* sceneMgr)
{
    // Attach the mesh to an entity and a scene node
    ogreEntity = sceneMgr->createEntity( getOgreName(), getOgreName()+".mesh" );
    //ogreEntity->setMaterialName(materialname.getValue());
    ogreEntity->setNormaliseNormals(true);
    ogreNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
    ogreNode->attachObject(ogreEntity);
    //ogreNode->setScale(Ogre::Vector3(10,10,10));
    ogreNode->setInitialState();
    //ogreNode->showBoundingBox(true);
}

void OgreVisualModel::update()
{
    if (topology && topology->getRevision() != meshRevision)
        uploadIndices();
    if (modified)
    {
        computeBounds();
        computeNormals();
        uploadVertices();
        modified = false;
    }
}

void OgreVisualModel::uploadVertices()
{
    Ogre::VertexData* vertexData = ogreMesh->sharedVertexData;
    Ogre::VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
    if (!ogreMesh->isPreparedForShadowVolumes())
    {
        if (x.size() > vertexData->vertexCount)
        {
            // need to resize hardware buffers
            if (x.size() >= 2* vertexData->vertexCount)
                vertexData->vertexCount = x.size();
            else
                vertexData->vertexCount *= 2;
            vBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), (vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, false);
            Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
            binding->setBinding(0, vBuffer);
        }

        Coord* pVertex = static_cast<Coord*>(vBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
        for (unsigned int i=0; i < x.size(); i++)
        {
            *(pVertex++) = x[i];
            *(pVertex++) = normals[i];
        }
        vBuffer->unlock();
    }
    else
    {
        // The mesh has been reorganized to be extruded for shadow volumes
        const Ogre::VertexElement* posElem = ogreMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
        Ogre::HardwareVertexBufferSharedPtr vBufferPos = ogreMesh->sharedVertexData->vertexBufferBinding->getBuffer(posElem->getSource());
        
        const Ogre::VertexElement* normElem = ogreMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
        Ogre::HardwareVertexBufferSharedPtr vBufferNorm = ogreMesh->sharedVertexData->vertexBufferBinding->getBuffer(normElem->getSource());

        if (x.size() > vertexData->vertexCount)
        {
            // need to resize hardware buffers
            if (x.size() >= 2* vertexData->vertexCount)
                vertexData->vertexCount = x.size();
            else
                vertexData->vertexCount *= 2;
            vBufferPos = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(posElem->getSource()), 2*(vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, false);
            vBufferNorm = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(normElem->getSource()), (vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, false);
            Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
            binding->setBinding(posElem->getSource(), vBufferPos);
            binding->setBinding(normElem->getSource(), vBufferNorm);
        }

        Coord* pVertex = static_cast<Coord*>(vBufferPos->lock(Ogre::HardwareBuffer::HBL_DISCARD));
        // Two copies of each position must be sent
        memcpy(pVertex, x.ptr(), x.size()*sizeof(Coord));
        memcpy(pVertex+x.size(), x.ptr(), x.size()*sizeof(Coord));
        vBufferPos->unlock();
        Coord* pNormal = static_cast<Coord*>(vBufferNorm->lock(Ogre::HardwareBuffer::HBL_DISCARD));
        memcpy(pNormal, normals.ptr(), normals.size()*sizeof(Coord));
        vBufferNorm->unlock();
        
        // We must recompute normals
        ogreMesh->getEdgeList()->updateFaceNormals(0,vBufferPos);
    }
}

void OgreVisualModel::uploadIndices()
{
    unsigned int nindices = 3*getNbTriangles();
    if (iBuffer->getNumIndexes() < nindices)
    {
        unsigned int nalloc;
        // need to resize hardware buffers
        if (nindices >= 2* iBuffer->getNumIndexes())
            nalloc = nindices;
        else
            nalloc = 2*iBuffer->getNumIndexes();
        iBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer((nalloc < 0x10000) ? Ogre::HardwareIndexBuffer::IT_16BIT : Ogre::HardwareIndexBuffer::IT_32BIT, (nalloc>0?nalloc:1), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
        ogreMesh->getSubMesh(0)->indexData->indexBuffer = iBuffer;
    }
    ogreMesh->getSubMesh(0)->indexData->indexCount = 3 * getNbTriangles();
    
    if (topology)
    {
        const sofa::component::topology::MeshTopology::SeqTriangles& triangles = topology->getTriangles();
        const sofa::component::topology::MeshTopology::SeqQuads& quads = topology->getQuads();
        if (iBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
        {
            unsigned int* pIndices = static_cast<unsigned int*>(iBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
            for (unsigned int i=0; i < triangles.size(); ++i)
            {
                *(pIndices++) = static_cast<unsigned int>(triangles[i][0]);
                *(pIndices++) = static_cast<unsigned int>(triangles[i][1]);
                *(pIndices++) = static_cast<unsigned int>(triangles[i][2]);
            }
            for (unsigned int i = 0; i < quads.size() ; i++)
            {
                *(pIndices++) = static_cast<unsigned int>(quads[i][0]);
                *(pIndices++) = static_cast<unsigned int>(quads[i][1]);
                *(pIndices++) = static_cast<unsigned int>(quads[i][2]);
                *(pIndices++) = static_cast<unsigned int>(quads[i][0]);
                *(pIndices++) = static_cast<unsigned int>(quads[i][2]);
                *(pIndices++) = static_cast<unsigned int>(quads[i][3]);
            }
            iBuffer->unlock();
        }
        else
        {
            unsigned short* pIndices = static_cast<unsigned short*>(iBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
            for (unsigned int i=0; i < triangles.size(); ++i)
            {
                *(pIndices++) = static_cast<unsigned short>(triangles[i][0]);
                *(pIndices++) = static_cast<unsigned short>(triangles[i][1]);
                *(pIndices++) = static_cast<unsigned short>(triangles[i][2]);
            }
            for (unsigned int i = 0; i < quads.size() ; i++)
            {
                *(pIndices++) = static_cast<unsigned short>(quads[i][0]);
                *(pIndices++) = static_cast<unsigned short>(quads[i][1]);
                *(pIndices++) = static_cast<unsigned short>(quads[i][2]);
                *(pIndices++) = static_cast<unsigned short>(quads[i][0]);
                *(pIndices++) = static_cast<unsigned short>(quads[i][2]);
                *(pIndices++) = static_cast<unsigned short>(quads[i][3]);
            }
            iBuffer->unlock();
        }
        meshRevision = topology->getRevision();
    }
    else
    {
        if (iBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
        {
            iBuffer->writeData(0, triangles.size() * sizeof(Triangle), triangles.ptr());
        }
        else
        {
            unsigned short* pIndices = static_cast<unsigned short*>(iBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
            for (unsigned int i=0; i < triangles.size(); ++i)
            {
                *(pIndices++) = static_cast<unsigned short>(triangles[i][0]);
                *(pIndices++) = static_cast<unsigned short>(triangles[i][1]);
                *(pIndices++) = static_cast<unsigned short>(triangles[i][2]);
            }
            iBuffer->unlock();
        }
    }
}

void OgreVisualModel::computeNormals()
{
    if (vertNormIdx.size() != x.size())
    {
        // x has been resized
        vertNormIdx.resize(x.size());
        for (unsigned int i = 0; i < x.size(); i++)
            vertNormIdx[i] = i;
    }
    normals.resize(x.size());
    for (unsigned int i = 0; i < normals.size(); i++)
        normals[i].clear();
    
    if (topology)
    {
        const sofa::component::topology::MeshTopology::SeqTriangles& triangles = topology->getTriangles();
        const sofa::component::topology::MeshTopology::SeqQuads& quads = topology->getQuads();
        for (unsigned int i = 0; i < triangles.size() ; i++)
        {
            const Coord & v1 = x[triangles[i][0]];
            const Coord & v2 = x[triangles[i][1]];
            const Coord & v3 = x[triangles[i][2]];
            Coord n = cross(v2-v1, v3-v1);
            n.normalize();
            normals[vertNormIdx[triangles[i][0]]] += n;
            normals[vertNormIdx[triangles[i][1]]] += n;
            normals[vertNormIdx[triangles[i][2]]] += n;
        }
        for (unsigned int i = 0; i < quads.size() ; i++)
        {
            const Coord & v1 = x[quads[i][0]];
            const Coord & v2 = x[quads[i][1]];
            const Coord & v3 = x[quads[i][3]];
            Coord n = cross(v2-v1, v3-v1);
            n.normalize();
            normals[vertNormIdx[quads[i][0]]] += n;
            normals[vertNormIdx[quads[i][1]]] += n;
            normals[vertNormIdx[quads[i][2]]] += n;
            normals[vertNormIdx[quads[i][3]]] += n;
        }
    }
    else
    {
        for (unsigned int i = 0; i < triangles.size() ; i++)
        {
            const Coord & v1 = x[triangles[i][0]];
            const Coord & v2 = x[triangles[i][1]];
            const Coord & v3 = x[triangles[i][2]];
            Coord n = cross(v2-v1, v3-v1);
            n.normalize();
            normals[vertNormIdx[triangles[i][0]]] += n;
            normals[vertNormIdx[triangles[i][1]]] += n;
            normals[vertNormIdx[triangles[i][2]]] += n;
        }
    }
    for (int i = normals.size()-1; i >=0 ; i--)
    {
        Coord n = normals[vertNormIdx[i]];
        n.normalize();
        normals[i] = n;
    }
}

void OgreVisualModel::computeBounds()
{
    Ogre::Vector3 bbmin, bbmax;
    float r2max = 0;
    if (x.size()>0)
    {
        bbmin = conv(x[0]);
        bbmax = bbmin;
        r2max = x[0].norm2();
        for (unsigned int i=1; i<x.size(); i++)
        {
            Ogre::Vector3 p = conv(x[i]);
            bbmin.makeFloor(p);
            bbmax.makeCeil(p);
            float r2 = p.squaredLength();
            if (r2 > r2max) r2max = r2;
        }
    }
    else
    {
        bbmin = Ogre::Vector3(0,0,0);
        bbmax = Ogre::Vector3(0,0,0);
    }
    ogreMesh->_setBounds( Ogre::AxisAlignedBox( bbmin, bbmax ), false );
    ogreMesh->_setBoundingSphereRadius(Ogre::Math::Sqrt(r2max));
    if (ogreNode)
        ogreNode->_updateBounds();
}

int OgreVisualModelClass = sofa::core::RegisterObject("OGRE 3D Visual Model")
.add < OgreVisualModel >()
;
