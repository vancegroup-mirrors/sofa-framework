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
#ifndef OGREVISUALMODEL_H
#define OGREVISUALMODEL_H

#include <Ogre.h>
#include <sofa/gui/qt/viewer/qtogre/DotSceneLoader.h>

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/VisualModel.h>
#include <sofa/core/componentmodel/behavior/MappedModel.h>
#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/component/visualmodel/VisualModelImpl.h>


namespace sofa
{

  namespace component
  {

    namespace visualmodel
    {

class OgreVisualModel : public sofa::component::visualmodel::VisualModelImpl
{
public:
    OgreVisualModel();
    ~OgreVisualModel();
    void setOgreSceneManager(Ogre::SceneManager* m){mSceneMgr=m;}
 private:
    virtual void internalDraw();
 public:
    virtual void reinit();
    virtual void initVisual(){internalDraw();}
    virtual void initTextures(){internalDraw();}

    virtual void applyUVTransformation();

protected:
    void uploadStructure();
    void uploadNormals();
    void updateMaterial();
    void convertManualToMesh();

    static int meshName;   
    static int materialName;   

    Data< std::string > materialFile;
    Data< bool > culling;


    std::string currentName;
    Ogre::ManualObject *ogreObject;
    Ogre::ManualObject *ogreNormalObject;
    Ogre::SceneManager* mSceneMgr;
    Ogre::MaterialPtr currentMaterial;
    Ogre::MaterialPtr currentMaterialNormals;

};
    }
  }
}
#endif
