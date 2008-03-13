#ifndef OGREVISUALMODEL_H
#define OGREVISUALMODEL_H

#include <Ogre.h>
#include "DotSceneLoader.h"

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

/// Resizable custom vector class.
template<class T>
class ResizableExtVector : public sofa::defaulttype::ExtVector<T>
{
public:
    typedef typename sofa::defaulttype::ExtVector<T>::size_type size_type;

    ~ResizableExtVector()
    {
        if (this->data!=NULL) delete[] this->data;
    }
    T* getData() { return this->data; }
    const T* getData() const { return this->data; }
    virtual void resize(size_type size)
    {
        if (size > this->maxsize)
        {
            T* oldData = this->data;
            this->maxsize = (size > 2*this->maxsize ? size : 2*this->maxsize);
            this->data = new T[this->maxsize];
            for (size_type i = 0 ; i < this->cursize ; ++i)
                this->data[i] = oldData[i];
            if (oldData!=NULL) delete[] oldData;
        }
        this->cursize = size;
    }
    void push_back(const T& v)
    {
        int i = this->size();
        resize(i+1);
        (*this)[i] = v;
    }
    T* ptr() { return this->data; }
    const T* ptr() const { return this->data; }
};

class OgreVisualModel : public sofa::core::VisualModel, public sofa::core::componentmodel::behavior::MappedModel<sofa::defaulttype::ExtVec3fTypes>
{
public:
    typedef sofa::defaulttype::ExtVec3fTypes DataTypes;
    typedef DataTypes::Coord Coord;
    typedef DataTypes::Deriv Deriv;
    typedef DataTypes::VecCoord VecCoord;
    typedef DataTypes::VecDeriv VecDeriv;
    
    DataField<std::string> filename;
    DataField<std::string> texturename;
    DataField<std::string> materialname;
    
    ResizableExtVector<Coord> x;
    ResizableExtVector<Coord> normals;
    
    typedef sofa::helper::fixed_array<int, 3> Triangle;
    ResizableExtVector<Triangle> triangles; ///< Only used if no external topology (i.e. topology==NULL)

    sofa::component::topology::MeshTopology* topology;
    int meshRevision;

    int getNbTriangles()
    {
        if (topology) return topology->getNbTriangles() + 2*topology->getNbQuads();
        else          return triangles.size();
    }

    /// This vector store which normal index is used for each vertice (as a single normal can be shared by multiple vertices, if texture coordinates are different)
    ResizableExtVector<int> vertNormIdx;
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

protected:

    void uploadVertices();

    void uploadIndices();

    void computeNormals();

    void computeBounds();
};

#endif
