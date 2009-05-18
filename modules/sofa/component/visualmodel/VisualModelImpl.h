/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_VISUALMODEL_VISUALMODELIMPL_H
#define SOFA_COMPONENT_VISUALMODEL_VISUALMODELIMPL_H

#include <string>
#include <sofa/core/VisualModel.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/componentmodel/behavior/MappedModel.h>
#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/helper/io/Mesh.h>
#include <sofa/component/component.h>

#include <map>

namespace sofa
{

namespace component
{

namespace visualmodel
{

using namespace sofa::defaulttype;


class RigidMappedModel : public core::componentmodel::behavior::MappedModel< Rigid3fTypes >
{
public:
    VecCoord xforms;
    bool xformsModified;

    RigidMappedModel()
        : xformsModified(false)
    {
    }

    const VecCoord* getX()  const { return &xforms; }
    const VecDeriv* getV()  const { return NULL; }

    VecCoord* getX()  { xformsModified = true; return &xforms;   }
    VecDeriv* getV()  { return NULL; }

    const VecCoord* getRigidX()  const { return getX(); }
    VecCoord* getRigidX()  { return getX(); }
};

class ExtVec3fMappedModel : public core::componentmodel::behavior::MappedModel< ExtVec3fTypes >
{
public:
    ResizableExtVector<Coord>* inputVertices;
    bool modified; ///< True if input vertices modified since last rendering

    ExtVec3fMappedModel()
        : inputVertices(NULL), modified(false)
    {
    }

    const VecCoord* getX()  const { return inputVertices; }
    const VecDeriv* getV()  const { return NULL; }

    VecCoord* getX()  { modified = true; return inputVertices; }
    VecDeriv* getV()  { return NULL; }

    const VecCoord* getVecX()  const { return getX(); }
    VecCoord* getVecX()  { return getX(); }
};

/**
 *  \brief Abstract class which implements partially VisualModel.
 *
 *  This class implemented all non-hardware (i.e OpenGL or DirectX)
 *  specific functions for rendering. It takes a 3D model (basically a .OBJ model)
 *  and apply transformations on it.
 *  At the moment, it is only implemented by OglModel for OpenGL systems.
 *
 */

class SOFA_COMPONENT_VISUALMODEL_API VisualModelImpl : public core::VisualModel, public ExtVec3fMappedModel, public RigidMappedModel
{
protected:
    // use types from ExtVec3fTypes

    typedef ExtVec3fTypes::Real Real;
    typedef ExtVec3fTypes::Coord Coord;
    typedef ExtVec3fTypes::VecCoord VecCoord;
    typedef ExtVec3fTypes::Deriv Deriv;
    typedef ExtVec3fTypes::VecDeriv VecDeriv;


    typedef Vec<2, float> TexCoord;
    typedef helper::fixed_array<int, 3> Triangle;
    typedef helper::fixed_array<int, 4> Quad;
    //typedef helper::fixed_array<int, 4> Tetrahedron;

    //ResizableExtVector<Coord>* inputVertices;

    bool useTopology; ///< True if list of facets should be taken from the attached topology
    int lastMeshRev; ///< Time stamps from the last time the mesh was updated from the topology
    bool useNormals; ///< True if normals should be read from file
    bool castShadow; ///< True if object cast shadows

    sofa::core::componentmodel::topology::BaseMeshTopology* _topology;

    /*     Data< ResizableExtVector<Coord> > vertices; */
    DataPtr< ResizableExtVector<Coord> > field_vertices;
    ResizableExtVector<Coord> vertices;

    DataPtr< ResizableExtVector<Coord> > field_vnormals;
    ResizableExtVector<Coord> vnormals;
    DataPtr< ResizableExtVector<TexCoord> > field_vtexcoords;
    ResizableExtVector<TexCoord> vtexcoords;

    DataPtr< ResizableExtVector<Triangle> > field_triangles;
    ResizableExtVector<Triangle> triangles;
    DataPtr< ResizableExtVector<Quad> > field_quads;
    ResizableExtVector<Quad> quads;

    /// If vertices have multiple normals/texcoords, then we need to separate them
    /// This vector store which input position is used for each vertice
    /// If it is empty then each vertex correspond to one position
    ResizableExtVector<int> vertPosIdx;

    /// Similarly this vector store which input normal is used for each vertice
    /// If it is empty then each vertex correspond to one normal
    ResizableExtVector<int> vertNormIdx;



    sofa::core::objectmodel::DataFileName fileMesh;
    sofa::core::objectmodel::DataFileName texturename;
    Data< Vector3 > translation;
    Data< Vector3 > rotation;
    Data< SReal > scale;

    Data< TexCoord >  scaleTex;
    Data< TexCoord >  translationTex;

    Vec3f bbox[2];

    virtual void internalDraw()
    {}

public:
    Data< sofa::helper::io::Mesh::Material > material;
    Data< bool > putOnlyTexCoords;

    VisualModelImpl();

    ~VisualModelImpl();

    void parse(core::objectmodel::BaseObjectDescription* arg);

    bool isTransparent();

    void drawVisual();
    void drawTransparent();
    void drawShadow();

    virtual bool loadTexture(const std::string& /*filename*/) { return false; }

    bool load(const std::string& filename, const std::string& loader, const std::string& textureName);

    void applyTranslation(const double dx, const double dy, const double dz);
    //Apply Rotation from Euler angles (in degree!)
    void applyRotation (const double rx, const double ry, const double rz);
    void applyRotation(const Quat q);
    void applyScale(const double s);
    virtual void applyUVTransformation();
    void applyUVTranslation(const double dU, const double dV);
    void applyUVScale(const double su, const double sv);

    void flipFaces();

    void setFilename(std::string s) {fileMesh.setValue(s);}
    void setTranslation(double dx,double dy,double dz) {translation.setValue(Vector3(dx,dy,dz));};
    void setRotation(double rx,double ry,double rz) {rotation.setValue(Vector3(rx,ry,rz));};
    void setScale(double s) {scale.setValue(s);};

    std::string getFilename() {return fileMesh.getValue();}

    void setColor(float r, float g, float b, float a);
    void setColor(std::string color);

    void setUseNormals(bool val) { useNormals = val;  }
    bool getUseNormals() const   { return useNormals; }

    void setCastShadow(bool val) { castShadow = val;  }
    bool getCastShadow() const   { return castShadow; }

    void setMesh(helper::io::Mesh &m, bool tex=false);
    bool isUsingTopology() const {return useTopology;};

    ResizableExtVector<Coord> * getVertices() {return &vertices;}
    void setVertices(ResizableExtVector<Coord> * x) {vertices = *x;}
    ResizableExtVector<Triangle> * getTriangles() {return &triangles;}
    void setTriangles(ResizableExtVector<Triangle> * t) {triangles = *t;}

    virtual void computePositions();
    virtual void computeMesh();
    virtual void computeNormals();
    virtual void computeBBox();

    virtual void updateBuffers() { };

    virtual void updateVisual();

    // handle topological changes
    virtual void handleTopologyChange();

    void init();

    void initVisual();

    bool addBBox(double* minBBox, double* maxBBox);

    //const VecCoord* getX()  const; // { return &x;   }
    //const VecDeriv* getV()  const { return NULL; }

    //VecCoord* getX(); //  { return &x;   }
    //VecDeriv* getV()  { return NULL; }

    /// Append this mesh to an OBJ format stream.
    /// The number of vertices position, normal, and texture coordinates already written is given as parameters
    /// This method should update them
    virtual void exportOBJ(std::string name, std::ostream* out, std::ostream* mtl, int& vindex, int& nindex, int& tindex);

    virtual std::string getTemplateName() const
    {
        return ExtVec3fMappedModel::getTemplateName();
    }

    static std::string templateName(const VisualModelImpl* p = NULL)
    {
        return ExtVec3fMappedModel::templateName(p);
    }
};

//typedef Vec<3,GLfloat> GLVec3f;
//typedef ExtVectorTypes<GLVec3f,GLVec3f> GLExtVec3fTypes;

} // namespace visualmodel

} // namespace component

} // namespace sofa

#endif
