/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
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

      int OgreVisualModel::meshName=0; //static counter to get unique name for entities
      int OgreVisualModel::materialName=0; 

      OgreVisualModel::OgreVisualModel():
	materialFile(initData(&materialFile,"materialOgre", "Name of the material used by Ogre")),
	culling(initData(&culling,true, "culling", "Activate Back-face culling in Ogre")),
	ogreObject(NULL), ogreNormalObject(NULL)
      {
      }


      OgreVisualModel::~OgreVisualModel()
      {
	if (mSceneMgr->hasEntity(currentName+"ENTITY"))
	  {	    
	    mSceneMgr->destroyEntity(currentName+"ENTITY");
	    Ogre::MeshManager::getSingleton().remove(currentName+"MESH");
	  }
      }
      void OgreVisualModel::init()
      {
        sofa::component::visualmodel::VisualModelImpl::init();

      }

      void OgreVisualModel::reinit()
      {
        sofa::component::visualmodel::VisualModelImpl::reinit();
	if (!currentMaterial.isNull())
	  {
	    updateMaterial();
	  }
      }

      bool OgreVisualModel::loadTexture(const std::string& filename)
      {      
        std::string file=filename;
        sofa::helper::system::DataRepository.findFile(file);
        Inherit::loadTexture(file);
        return true;
      }

      void OgreVisualModel::updateMaterial()
      {
	currentMaterial->setReceiveShadows(true); 
	currentMaterial->getTechnique(0)->getPass(0)->setLightingEnabled(true);
	
	
	if (this->material.getValue().useDiffuse)	  
	    currentMaterial->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(this->material.getValue().diffuse[0],this->material.getValue().diffuse[1],this->material.getValue().diffuse[2],this->material.getValue().diffuse[3]));	 
	else
	  currentMaterial->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(0,0,0,0));

	if (this->material.getValue().useAmbient)
	  currentMaterial->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(this->material.getValue().ambient[0],this->material.getValue().ambient[1],this->material.getValue().ambient[2],this->material.getValue().ambient[3]));
	else
	  currentMaterial->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(0,0,0,0));

	if (this->material.getValue().useEmissive)
	  currentMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(Ogre::ColourValue(this->material.getValue().emissive[0],this->material.getValue().emissive[1],this->material.getValue().emissive[2],this->material.getValue().emissive[3]));
	else
	  currentMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(Ogre::ColourValue(0,0,0,0));

	if (this->material.getValue().useSpecular)
	  currentMaterial->getTechnique(0)->getPass(0)->setSpecular(Ogre::ColourValue(this->material.getValue().specular[0],this->material.getValue().specular[1],this->material.getValue().specular[2],this->material.getValue().specular[3]));
	else
	  currentMaterial->getTechnique(0)->getPass(0)->setSpecular(Ogre::ColourValue(0,0,0,0));

	if (this->material.getValue().useShininess)
	  currentMaterial->getTechnique(0)->getPass(0)->setShininess(Ogre::Real(this->material.getValue().shininess));
	else
	  currentMaterial->getTechnique(0)->getPass(0)->setShininess(Ogre::Real(45));

	if ( (this->material.getValue().useDiffuse && this->material.getValue().diffuse[3] < 1) || 
	     (this->material.getValue().useAmbient && this->material.getValue().ambient[3] < 1) )
	  {
	    currentMaterial->setDepthWriteEnabled(false);
	    currentMaterial->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	    currentMaterial->setCullingMode(Ogre::CULL_NONE); 
	  }

	currentMaterial->compile();
      }

      void OgreVisualModel::uploadStructure()
      {
	bool hasTexCoords=vtexcoords.size();

// 	std::ofstream s((this->getName() + std::string(".obj")).c_str());

	for (unsigned int i=0;i<vertices.size();++i)
	  {
	    ogreObject->position(vertices[i][0],vertices[i][1],vertices[i][2]);
	    ogreObject->normal(vnormals[i][0],vnormals[i][1],vnormals[i][2]);
	    if (hasTexCoords) ogreObject->textureCoord(vtexcoords[i][0],vtexcoords[i][1]);
//          s << hasTexCoords << " v[" << i << "]" << vtexcoords[i][0] << " !!! " <<  vtexcoords[i][1] << std::endl;

	  }

	for (unsigned int i=0;i<triangles.size();++i)
	  {
	    ogreObject->triangle(triangles[i][0],triangles[i][1],triangles[i][2]);
// 	    s << "f " << triangles[i][2]+1 << " " << triangles[i][1]+1 << " " << triangles[i][0]+1 << "\n";
	  }
	for (unsigned int i=0;i<quads.size();++i)
	  {
	    ogreObject->quad(quads[i][0],quads[i][1],quads[i][2],quads[i][3]);
	  }
// 	s.close();
      }

      void OgreVisualModel::uploadNormals()
      {       
	for (unsigned int i = 0; i < vertices.size(); i++)
	  {
	    ogreNormalObject->position(vertices[i][0],vertices[i][1],vertices[i][2]);
	    ogreNormalObject->position(vertices[i][0]+vnormals[i][0],vertices[i][1]+vnormals[i][1],vertices[i][2]+vnormals[i][2]);
	    ogreNormalObject->index(2*i);
	    ogreNormalObject->index(2*i+1);
	  }
      }
      void OgreVisualModel::convertManualToMesh()
      {
	if (mSceneMgr->hasEntity(currentName+"ENTITY"))
	  {	    
	    mSceneMgr->destroyEntity(currentName+"ENTITY");
	    Ogre::MeshManager::getSingleton().remove(currentName+"MESH");
	  }
	
	Ogre::MeshPtr ogreMesh = ogreObject->convertToMesh(currentName+"MESH", "General");
	Ogre::Entity *e = mSceneMgr->createEntity(currentName+"ENTITY", ogreMesh->getName());
 	mSceneMgr->getRootSceneNode()->attachObject(e);	
      }

    void OgreVisualModel::internalDraw(bool /*transparent*/)
      {
	if (!ogreObject)
	  {
	    //If visual model is empty, return
	    if (triangles.size() == 0 && quads.size() == 0) return;


	    //Create the Visual Model
	    std::ostringstream s;
	    s << "OgreVisualModel["<<meshName<<"]";
	    currentName=s.str();
	    ogreObject = (Ogre::ManualObject *) mSceneMgr->createMovableObject(s.str(),"ManualObject");
	    ogreObject->setDynamic(true);
	    ogreObject->setCastShadows(true);

	    //Create a model for the normals
	    s.str("");
	    s << "OgreVisualNormalModel["<<meshName++<<"]";

	    ogreNormalObject = (Ogre::ManualObject *) mSceneMgr->createMovableObject(s.str(),"ManualObject");
	    ogreNormalObject->setDynamic(true);
	    ogreNormalObject->setCastShadows(false);
	    mSceneMgr->getRootSceneNode()->attachObject(ogreNormalObject);	



	    //Create the materials
	    if (!materialFile.getValue().empty())
	      {
                currentMaterial = Ogre::MaterialManager::getSingleton().getByName(materialFile.getFullPath());
	      }
	    if (currentMaterial.isNull())
	      {
		//Create the Material for the object
		s.str("");
		s << "OgreVisualMaterial[" << materialName << "]" ;
		currentMaterial = Ogre::MaterialManager::getSingleton().create(s.str(), "General");  
		//If a texture is specified
		if (!texturename.getValue().empty() && Ogre::ResourceGroupManager::getSingleton().resourceExists("General",texturename.getValue()) )
		  {
		    currentMaterial->getTechnique(0)->getPass(0)->createTextureUnitState(texturename.getValue());
		  }
		if (!culling.getValue()) currentMaterial->getTechnique(0)->getPass(0)->setCullingMode(Ogre::CULL_NONE);

		//Create the Material to draw the normals
		s.str("");
		s << "OgreNormalMaterial[" << materialName++ << "]" ;
		currentMaterialNormals = Ogre::MaterialManager::getSingleton().create(s.str(), "General"); 
	
  		currentMaterialNormals->setLightingEnabled(false);
	      }
	    updateMaterial();


	    //Upload the indices
	    ogreObject->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST); 
	    uploadStructure();
	    ogreObject->end();


	    if (getContext()->getShowNormals())
	      {	
		ogreNormalObject->begin(currentMaterialNormals->getName(), Ogre::RenderOperation::OT_LINE_LIST); 
		uploadNormals();
		ogreNormalObject->end();
	      }
	  }
	else
	  {
	    if (getContext()->getShowWireFrame() || !culling.getValue()) 
	      currentMaterial->getTechnique(0)->setCullingMode(Ogre::CULL_NONE);
	    else	     
	      currentMaterial->getTechnique(0)->setCullingMode(Ogre::CULL_CLOCKWISE);
	      
	    
	    //Visual Model update
	    ogreObject->beginUpdate(0);  
	    uploadStructure();      
	    ogreObject->end();
	    if (vertices.size() != 0)
	      convertManualToMesh();

	    //Normals
	    if (ogreNormalObject->getNumSections())
	      {
		ogreNormalObject->beginUpdate(0);
		uploadNormals();		 
		ogreNormalObject->end();
	      }
	    else
	      {
		ogreNormalObject->begin(currentMaterialNormals->getName(), Ogre::RenderOperation::OT_LINE_LIST); 
		uploadNormals();
		ogreNormalObject->end();
	      }

	  }


	if (mSceneMgr->hasEntity(currentName+"ENTITY"))
	  {
	    Ogre::Entity *e=mSceneMgr->getEntity(currentName+"ENTITY");
	    if (!this->getContext()->getShowVisualModels())
	      {
		if (e->isAttached()) mSceneMgr->getRootSceneNode()->detachObject(e);
	      }
	    else
	      {
		if (!e->isAttached()) mSceneMgr->getRootSceneNode()->attachObject(e);
	      }

	    ogreNormalObject->setVisible(this->getContext()->getShowNormals());
	  }
      }

      void OgreVisualModel::applyUVTransformation()
      {	
// 	for (unsigned int i=0;i<vtexcoords.size();++i)
// 	  {
// 	    vtexcoords[i][0] = vtexcoords[i][0];
// 	    vtexcoords[i][1] = 1-vtexcoords[i][1];
// 	  }
	this->applyUVScale(scaleTex.getValue()[0], scaleTex.getValue()[1]);
	this->applyUVTranslation(translationTex.getValue()[0],translationTex.getValue()[1]);
 	scaleTex.setValue(TexCoord(1,1));
 	translationTex.setValue(TexCoord(0,0));
      }
      int OgreVisualModelClass = sofa::core::RegisterObject("OGRE 3D Visual Model")
	.add < OgreVisualModel >();

    }
  }
}
