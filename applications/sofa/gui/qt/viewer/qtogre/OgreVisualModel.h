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
#ifndef OGREVISUALMODEL_H
#define OGREVISUALMODEL_H

#include <Ogre.h>
#include <sofa/gui/qt/viewer/qtogre/DotSceneLoader.h>

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/VisualModel.h>
#include <sofa/core/componentmodel/behavior/MappedModel.h>
#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/component/visualmodel/VisualModelImpl.h>
// SOFA -> OGRE3D conversion

namespace sofa
{

  namespace component
  {

    namespace visualmodel
    {
template<class T>
Ogre::Vector3 conv(const sofa::defaulttype::Vec<3,T>& v)
{
    return Ogre::Vector3((float)v[0], (float)v[1], (float)v[2]);
}

// SOFA <- OGRE3D conversion

template<class T>
sofa::defaulttype::Vec3f conv(const Ogre::Vector3& v)
{
    return sofa::defaulttype::Vec3f((float)v[0], (float)v[1], (float)v[2]);
}

class OgreVisualModel : public sofa::component::visualmodel::VisualModelImpl//, public sofa::core::componentmodel::behavior::MappedModel<sofa::defaulttype::ExtVec3fTypes>
{
public:
  
    static bool lightSwitched;
    Data<std::string> materialname;
       

    //Initial Position of a OgreVisualModel
    float dx,dy,dz,scale;
    
    int meshRevision;

    int getNbTriangles()
    {
      sofa::core::componentmodel::topology::BaseMeshTopology* topology = dynamic_cast<sofa::core::componentmodel::topology::BaseMeshTopology*>(getContext()->getTopology());
        if (useTopology && topology) 
	{	  
	  return topology->getNbTriangles() + 2*topology->getNbQuads();
	}
        else          return (this->triangles.size() + 2*this->quads.size());
    }    

    Ogre::MeshPtr ogreMesh;
    Ogre::Entity* ogreEntity;
    Ogre::SceneNode* ogreNode;
    
    Ogre::HardwareVertexBufferSharedPtr vBuffer;
    Ogre::HardwareIndexBufferSharedPtr iBuffer;
    
    OgreVisualModel();
    
    std::string getOgreName();
        
   
    void parse(core::objectmodel::BaseObjectDescription* );

    void reinit();
    
    virtual void attach(Ogre::SceneManager* sceneMgr);

    // GL method -> do nothing for Ogre
    virtual void draw() {}
    // GL method -> do nothing for Ogre
    virtual void initTextures() {}

    virtual void updateVisual();
    

    static int counter;
protected:
    ResizableExtVector<Vec<2,float> > mirror_tex;
    void uploadVertices();

    void uploadIndices();

    void computeNormals();

    void computeBounds();

    void updateMaterial(Ogre::MaterialPtr &mat);
    std::string uniqueName(void);
    std::string submesh_material;
};
    }
  }
}
#endif
