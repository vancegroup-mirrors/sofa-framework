#ifndef OGREVISUALMODEL_H
#define OGREVISUALMODEL_H

#include <Ogre.h>
#include "viewer/qtogre/DotSceneLoader.h"

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/core/VisualModel.h>
#include <sofa/core/componentmodel/behavior/MappedModel.h>
#include <sofa/component/topology/MeshTopology.h>

// SOFA -> OGRE3D conversion

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

class OgreVisualModel : public sofa::core::VisualModel, public sofa::core::componentmodel::behavior::MappedModel<sofa::defaulttype::ExtVec3fTypes>
{
public:
  
  static bool lightSwitched;

    typedef sofa::defaulttype::ExtVec3fTypes DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Deriv Deriv;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::VecDeriv VecDeriv;
    
    DataField<std::string> filename;
    DataField<std::string> texturename;
    DataField<std::string> materialname;
    DataField<std::string> colorname;
       
    float color[4];

    //Initial Position of a OgreVisualModel
    DataField<float> dx,dy,dz,scale,scaleTex;

     sofa::defaulttype::ResizableExtVector<Coord> x;
     sofa::defaulttype::ResizableExtVector<Coord> normals;
     sofa::defaulttype::ResizableExtVector< sofa::defaulttype::Vec2f > texcoords;
    
    typedef sofa::helper::fixed_array<int, 3> Triangle;
     sofa::defaulttype::ResizableExtVector<Triangle> triangles; ///< Only used if no external topology (i.e. topology==NULL)

    sofa::component::topology::MeshTopology* topology;
    int meshRevision;

    int getNbTriangles()
    {
        if (topology) return topology->getNbTriangles() + 2*topology->getNbQuads();
        else          return triangles.size();
    }

    /// This vector store which normal index is used for each vertice (as a single normal can be shared by multiple vertices, if texture coordinates are different)
     sofa::defaulttype::ResizableExtVector<int> vertNormIdx;
    int nbNormals;

    Ogre::MeshPtr ogreMesh;
    Ogre::Entity* ogreEntity;
    Ogre::SceneNode* ogreNode;
    
    bool useTexture;
    bool useNormals;
    bool useTopology; ///< True if list of facets should be taken from the attached topology
    bool modified;    ///< True if input vertices modified since last rendering
    
    Ogre::HardwareVertexBufferSharedPtr vBuffer;
    Ogre::HardwareIndexBufferSharedPtr iBuffer;
    
    OgreVisualModel();
    
    std::string getOgreName();
    
    virtual void init();
    
    virtual void attach(Ogre::SceneManager* sceneMgr);

    virtual VecCoord* getX() { modified = true; return &x; }
    virtual VecDeriv* getV() { return NULL; }

    virtual const VecCoord* getX()  const { return &x; }
    virtual const VecDeriv* getV()  const { return NULL; }

    // GL method -> do nothing for Ogre
    virtual void draw() {}
    // GL method -> do nothing for Ogre
    virtual void initTextures() {}

    virtual void update();
    
    void setColor(std::string color);

    static int counter;
protected:

    void uploadVertices();

    void uploadIndices();

    void computeNormals();

    void computeBounds();

    std::string uniqueName(void);

};

#endif
