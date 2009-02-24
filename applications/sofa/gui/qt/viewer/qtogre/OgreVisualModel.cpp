/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU General Public License as published by the Free  *
* Software Foundation; either version 2 of the License, or (at your option)   *
* any later version.                                                          *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for    *
* more details.                                                               *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc., 51  *
* Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.                   *
*******************************************************************************
*                            SOFA :: Applications                             *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/gui/qt/viewer/qtogre/OgreVisualModel.h>

#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/io/Mesh.h>
#include <sofa/simulation/tree/GNode.h>

#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/core/componentmodel/behavior/BaseMechanicalMapping.h>
#include <sofa/core/BaseMapping.h>

#include <iostream>

namespace sofa
{

  namespace component
  {

    namespace visualmodel
    {
      
SOFA_DECL_CLASS(OgreVisualModel)

int OgreVisualModel::counter; //static counter to get unique name for entities

OgreVisualModel::OgreVisualModel()
	: materialname(initData(&materialname, "materialname", "material name")),scale(1.0)
, ogreMesh(NULL), ogreEntity(NULL), ogreNode(NULL)
{	
}


std::string OgreVisualModel::uniqueName(void)
{
     std::ostringstream s;
      s << getOgreName() + "_" << ++OgreVisualModel::counter;
      return s.str();
}


std::string OgreVisualModel::getOgreName()
{
  return "SOFA"+static_cast<sofa::simulation::tree::GNode*>(getContext())->getPathName()+"/"+getName();
}

void OgreVisualModel::parse(core::objectmodel::BaseObjectDescription* arg)
{
  
  if (arg->getAttribute("scale")!=NULL) {
    //obj->applyScale(atof(arg->getAttribute("scale","1.0")));
    scale=atof(arg->getAttribute("scale","1.0"));
  }
//   if (arg->getAttribute("rx")!=NULL) {
//     //obj->applyRotation(Quat(Vector3(1,0,0), atof(arg->getAttribute("rx","0.0"))*R_PI/180));
//   }
//   if (arg->getAttribute("ry")!=NULL) {
//     //obj->applyRotation(Quat(Vector3(0,1,0), atof(arg->getAttribute("ry","0.0"))*R_PI/180));
//   }
//   if (arg->getAttribute("rz")!=NULL) {
//     //obj->applyRotation(Quat(Vector3(0,0,1), atof(arg->getAttribute("rz","0.0"))*R_PI/180));
//   }
    
  if (arg->getAttribute("dx")!=NULL || arg->getAttribute("dy")!=NULL || arg->getAttribute("dz")!=NULL) {
    //obj->applyTranslation(atof(arg->getAttribute("dx","0.0")),atof(arg->getAttribute("dy","0.0")),atof(arg->getAttribute("dz","0.0")));
    dx=atof(arg->getAttribute("dx","0.0"));
    dy=atof(arg->getAttribute("dy","0.0"));
    dz=atof(arg->getAttribute("dz","0.0"));
  }
  
  VisualModelImpl::parse(arg);  
}
bool OgreVisualModel::lightSwitched;

void OgreVisualModel::attach(Ogre::SceneManager* sceneMgr)
{
  // Attach the mesh to an entity and a scene node
  // Create OGRE Mesh
  ogreMesh = Ogre::MeshManager::getSingleton().getByName(getOgreName()+".mesh");
  if (!ogreMesh.isNull()) setName(uniqueName());
  ogreMesh = Ogre::MeshManager::getSingleton().createManual(getOgreName()+".mesh", "General"); 

  if (3*this->vertices.size() > sceneMgr->getShadowIndexBufferSize()) 
    sceneMgr->setShadowIndexBufferSize(3*this->vertices.size());

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
  if (!this->texturename.getValue().empty())
    {
      // two dimensional texture coordinates
      vertexDecl->addElement(1, currOffset[1], Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
      currOffset[1] += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
    }
  // allocate the vertex buffer
  vertexData->vertexCount = this->vertices.size();
  vBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), (vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, false);
  Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
  binding->setBinding(0, vBuffer);
    
  uploadVertices();
  // Upload tex coordinates
  if (!this->texturename.getValue().empty())
    {
      Ogre::HardwareVertexBufferSharedPtr vBuffer2 = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(1), (vertexData->vertexCount>0?vertexData->vertexCount:1), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
      Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
      binding->setBinding(1, vBuffer2);
      //texture coordinates are inverted with Ogre
      for (unsigned int i=0;i<this->vtexcoords.size();++i) this->vtexcoords[i][1] =1- this->vtexcoords[i][1];
      vBuffer2->writeData(0, this->vtexcoords.size() * sizeof(this->vtexcoords[0]), this->vtexcoords.getData());
    }
  submesh->useSharedVertices = true;
  // allocate index buffer
  submesh->indexData->indexCount = 3 * getNbTriangles();
  iBuffer = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer((this->vertices.size() < 0x10000) ? Ogre::HardwareIndexBuffer::IT_16BIT : Ogre::HardwareIndexBuffer::IT_32BIT, (submesh->indexData->indexCount>0?submesh->indexData->indexCount:1), Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
  submesh->indexData->indexBuffer = iBuffer;
  uploadIndices();
  computeBounds();

  //Apply the good material/texture/colour
  submesh_material = std::string("Sofa/") + uniqueName();
  Ogre::MaterialPtr mat;
  if (!materialname.getValue().empty()) {    
    Ogre::MaterialPtr mat_mem = Ogre::MaterialManager::getSingleton().getByName(materialname.getValue());
    mat = mat_mem->clone(submesh_material);
  }
  else
    {		
	  mat = Ogre::MaterialManager::getSingleton().create(submesh_material, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME); 
	  //Two side polygon!
	  mat->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);
	  
	  if (!this->texturename.getValue().empty()) //A texture has been specified
	    {
	      Ogre::TextureUnitState *ogre_texture = mat->getTechnique(0)->getPass(0)->createTextureUnitState(getOgreName()+std::string("_texture")); 
	      ogre_texture->setTextureName(this->texturename.getValue());
// 		  if (scaleTex.getValue() != 0)
// 			ogre_texture->setTextureScale(Ogre::Real(this->scaleTex.getValue()), Ogre::Real(this->scaleTex.getValue()));
	    }
    }       
    submesh->setMaterialName(submesh_material);
    updateMaterial(mat);

// unsigned short src, dest;
//  if (!ogreMesh->suggestTangentVectorBuildParams(Ogre::VES_TANGENT,src, dest))
// {
//   ogreMesh->buildTangentVectors(Ogre::VES_TANGENT,src, dest);
// }

  // this line makes clear the mesh is loaded (avoids memory leaks)

  ogreMesh->load();

  ogreEntity = sceneMgr->createEntity( getOgreName(), getOgreName()+".mesh" );
  
  ogreEntity->setNormaliseNormals(true); 

  ogreNode = sceneMgr->getRootSceneNode()->createChildSceneNode();
  
  ogreNode->attachObject(ogreEntity);

  ogreNode->setInitialState();
  
  //ogreNode->showBoundingBox(true);
}


void OgreVisualModel::reinit()
{
 Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().getByName(submesh_material);
 if (!mat.isNull())
 {
   updateMaterial(mat);
 }
 updateVisual();
}

void OgreVisualModel::updateMaterial(Ogre::MaterialPtr &mat)
{  
    if (this->material.getValue().useDiffuse)
      mat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(this->material.getValue().diffuse[0],this->material.getValue().diffuse[1],this->material.getValue().diffuse[2],this->material.getValue().diffuse[3]));
    else
      mat->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0,0,0,0));
    
    if (this->material.getValue().useAmbient)
      mat->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(this->material.getValue().ambient[0],this->material.getValue().ambient[1],this->material.getValue().ambient[2],this->material.getValue().ambient[3]));
    else
      mat->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(0,0,0,0));
    
    if (this->material.getValue().useEmissive)
      mat->getTechnique(0)->getPass(0)->setSelfIllumination(Ogre::ColourValue(this->material.getValue().emissive[0],this->material.getValue().emissive[1],this->material.getValue().emissive[2],this->material.getValue().emissive[3]));
    else
      mat->getTechnique(0)->getPass(0)->setSelfIllumination(Ogre::ColourValue(0,0,0,0));
    
    if (this->material.getValue().useSpecular)
      mat->getTechnique(0)->getPass(0)->setSpecular(Ogre::ColourValue(this->material.getValue().specular[0],this->material.getValue().specular[1],this->material.getValue().specular[2],this->material.getValue().specular[3]));
    else
      mat->getTechnique(0)->getPass(0)->setSpecular(Ogre::ColourValue(0,0,0,0));
    
    if (this->material.getValue().useShininess)
      mat->getTechnique(0)->getPass(0)->setShininess(Ogre::Real(this->material.getValue().shininess));
    else
      mat->getTechnique(0)->getPass(0)->setShininess(Ogre::Real(45));
    mat->compile();
}

void OgreVisualModel::updateVisual()
{
  if (ogreMesh.isNull()) return;
  sofa::core::componentmodel::topology::BaseMeshTopology* topology = dynamic_cast<sofa::core::componentmodel::topology::BaseMeshTopology*>(getContext()->getTopology());
  if (topology && topology->getRevision() != meshRevision)
    uploadIndices();


  if (this->modified//  || lightSwitched
      )
  { 
      uploadVertices(); 
      computeNormals();
      computeBounds();   
      lightSwitched = false;
    }
}


void OgreVisualModel::uploadVertices()
{
  this->modified = false;
  Ogre::VertexData* vertexData = ogreMesh->sharedVertexData;
  Ogre::VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
  
  if (!ogreMesh->isPreparedForShadowVolumes())
    {
      if (this->vertices.size() > vertexData->vertexCount)
        {
	  // need to resize hardware buffers
	  if (this->vertices.size() >= 2* vertexData->vertexCount)
	    vertexData->vertexCount = this->vertices.size();
	  else
	    vertexData->vertexCount *= 2;
	  
	  vBuffer = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), (vertexData->vertexCount>0?vertexData->vertexCount:1),	      
	      Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY, false);
	  
	  Ogre::VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	  
	  binding->setBinding(0, vBuffer);
        }
	if (this->vnormals.size() == 0) computeNormals();
      Coord* pVertex = static_cast<Coord*>(vBuffer->lock(Ogre::HardwareBuffer::HBL_DISCARD));
      for (unsigned int i=0; i < this->vertices.size(); i++)
        {
	  *(pVertex++) = this->vertices[i];
 	  if (this->vnormals.size() <= i)  ; //*(pVertex++) = (Real)0;
 	  else 	                           
	    *(pVertex++) = this->vnormals[i];
        }
      dx = dy = dz = 0;
      scale = 1;
      vBuffer->unlock();      
    }
  else
    {      

      // The mesh has been reorganized to be extruded for shadow volumes
      const Ogre::VertexElement* posElem = ogreMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
      Ogre::HardwareVertexBufferSharedPtr vBufferPos = ogreMesh->sharedVertexData->vertexBufferBinding->getBuffer(posElem->getSource());
        
      const Ogre::VertexElement* normElem = ogreMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
      Ogre::HardwareVertexBufferSharedPtr vBufferNorm = ogreMesh->sharedVertexData->vertexBufferBinding->getBuffer(normElem->getSource());

      if (this->vertices.size() > vertexData->vertexCount)
	{
	  // need to resize hardware buffers
	  if (this->vertices.size() >= 2* vertexData->vertexCount)
	    vertexData->vertexCount = this->vertices.size();
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
      memcpy(pVertex, this->vertices.getData(), this->vertices.size()*sizeof(Coord));
      memcpy(pVertex+this->vertices.size(), this->vertices.getData(), this->vertices.size()*sizeof(Coord));
      vBufferPos->unlock();
      Coord* pNormal = static_cast<Coord*>(vBufferNorm->lock(Ogre::HardwareBuffer::HBL_DISCARD));
      memcpy(pNormal, this->vnormals.getData(), this->vnormals.size()*sizeof(Coord));
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
    sofa::core::componentmodel::topology::BaseMeshTopology* topology = dynamic_cast<sofa::core::componentmodel::topology::BaseMeshTopology*>(getContext()->getTopology());
  
    if (this->useTopology && topology)
    {      
      const sofa::core::componentmodel::topology::BaseMeshTopology::SeqTriangles& triangles = topology->getTriangles();
        const sofa::core::componentmodel::topology::BaseMeshTopology::SeqQuads& quads = topology->getQuads();
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
	  iBuffer->writeData(0, this->triangles.size() * sizeof(Triangle), this->triangles.getData());
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
	      *(pIndices++) = static_cast<unsigned short>(this->quads[i][0]);
	      *(pIndices++) = static_cast<unsigned short>(this->quads[i][1]);
	      *(pIndices++) = static_cast<unsigned short>(this->quads[i][2]);
	      *(pIndices++) = static_cast<unsigned short>(this->quads[i][0]);
	      *(pIndices++) = static_cast<unsigned short>(this->quads[i][2]);
	      *(pIndices++) = static_cast<unsigned short>(this->quads[i][3]);
            }
            iBuffer->unlock();
        }
	
    }
    
}

void OgreVisualModel::computeNormals()
{
      if (this->vertNormIdx.size() != this->vertices.size())
      {
          // x has been resized
	this->vertNormIdx.resize(this->vertices.size());
	for (unsigned int i = 0; i < this->vertices.size(); i++)
              this->vertNormIdx[i] = i;
      }
      if (this->vertices.size() == 0) return; //nothing to do
      this->vnormals.resize(this->vertices.size());
       for (unsigned int i = 0; i < this->vnormals.size(); i++)
           this->vnormals[i].clear();
      if (this->useTopology)
      {
	
	  sofa::core::componentmodel::topology::BaseMeshTopology* topology = dynamic_cast<sofa::core::componentmodel::topology::BaseMeshTopology*>(getContext()->getTopology());
	  if (topology != NULL)
	  {
          const sofa::core::componentmodel::topology::BaseMeshTopology::SeqTriangles& triangles = topology->getTriangles();
          const sofa::core::componentmodel::topology::BaseMeshTopology::SeqQuads& quads = topology->getQuads();
          for (unsigned int i = 0; i < triangles.size() ; i++)
          {
	    const Coord & v1 = this->vertices[triangles[i][0]];
	    const Coord & v2 = this->vertices[triangles[i][1]];
	    const Coord & v3 = this->vertices[triangles[i][2]];
              Coord n = cross(v2-v1, v3-v1);
              n.normalize();
	      this->vnormals[vertNormIdx[triangles[i][0]]] += n;
	      this->vnormals[vertNormIdx[triangles[i][1]]] += n;
	      this->vnormals[vertNormIdx[triangles[i][2]]] += n;
          }
          for (unsigned int i = 0; i < quads.size() ; i++)
          {
	    const Coord & v1 = this->vertices[quads[i][0]];
	    const Coord & v2 = this->vertices[quads[i][1]];
	    const Coord & v3 = this->vertices[quads[i][3]];
              Coord n = cross(v2-v1, v3-v1);
              n.normalize();
	      this->vnormals[vertNormIdx[quads[i][0]]] += n;
	      this->vnormals[vertNormIdx[quads[i][1]]] += n;
	      this->vnormals[vertNormIdx[quads[i][2]]] += n;
	      this->vnormals[vertNormIdx[quads[i][3]]] += n;
          }
	  }
      }
      else
      {	  
          for (unsigned int i = 0; i < triangles.size() ; i++)
          {
	    const Coord & v1 = this->vertices[triangles[i][0]];
	    const Coord & v2 = this->vertices[triangles[i][1]];
	    const Coord & v3 = this->vertices[triangles[i][2]];
              Coord n = cross(v2-v1, v3-v1);
              n.normalize();
	      this->vnormals[vertNormIdx[triangles[i][0]]] += n;
	      this->vnormals[vertNormIdx[triangles[i][1]]] += n;
	      this->vnormals[vertNormIdx[triangles[i][2]]] += n;
          }
      }
      for (int i = this->vnormals.size()-1; i >=0 ; i--)
      {
	Coord n = this->vnormals[vertNormIdx[i]];
          n.normalize();
	  this->vnormals[i] = n;
      }
}

void OgreVisualModel::computeBounds()
{
    Ogre::Vector3 bbmin, bbmax;
    float r2max = 0;
    if (this->vertices.size()>0)
    {
      bbmin = conv(this->vertices[0]);
        bbmax = bbmin;
	r2max = this->vertices[0].norm2();
	for (unsigned int i=1; i<this->vertices.size(); i++)
        {
	  Ogre::Vector3 p = conv(this->vertices[i]);
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
    }
  }
}
