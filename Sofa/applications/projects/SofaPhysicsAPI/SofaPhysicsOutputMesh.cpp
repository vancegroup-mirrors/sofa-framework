#include "SofaPhysicsAPI.h"
#include "SofaPhysicsOutputMesh_impl.h"

SofaPhysicsOutputMesh::SofaPhysicsOutputMesh()
    : impl(new Impl)
{
}

SofaPhysicsOutputMesh::~SofaPhysicsOutputMesh()
{
    delete impl;
}

const char* SofaPhysicsOutputMesh::getName() ///< (non-unique) name of this object
{
    return impl->getName();
}

ID SofaPhysicsOutputMesh::getID() ///< unique ID of this object
{
    return impl->getID();
}

unsigned int SofaPhysicsOutputMesh::getNbVertices() ///< number of vertices
{
    return impl->getNbVertices();
}
const Real* SofaPhysicsOutputMesh::getVPositions()  ///< vertices positions (Vec3)
{
    return impl->getVPositions();
}
const Real* SofaPhysicsOutputMesh::getVNormals()    ///< vertices normals   (Vec3)
{
    return impl->getVNormals();
}
const Real* SofaPhysicsOutputMesh::getVTexCoords()  ///< vertices UVs       (Vec2)
{
    return impl->getVTexCoords();
}
int SofaPhysicsOutputMesh::getVerticesRevision()    ///< changes each time vertices data are updated
{
    return impl->getVerticesRevision();
}

unsigned int SofaPhysicsOutputMesh::getNbLines() ///< number of lines
{
    return impl->getNbLines();
}
const Index* SofaPhysicsOutputMesh::getLines()   ///< lines topology (2 indices / line)
{
    return impl->getLines();
}
int SofaPhysicsOutputMesh::getLinesRevision()    ///< changes each time lines data is updated
{
    return impl->getLinesRevision();
}

unsigned int SofaPhysicsOutputMesh::getNbTriangles() ///< number of triangles
{
    return impl->getNbTriangles();
}
const Index* SofaPhysicsOutputMesh::getTriangles()   ///< triangles topology (3 indices / triangle)
{
    return impl->getTriangles();
}
int SofaPhysicsOutputMesh::getTrianglesRevision()    ///< changes each time triangles data is updated
{
    return impl->getTrianglesRevision();
}

unsigned int SofaPhysicsOutputMesh::getNbQuads() ///< number of quads
{
    return impl->getNbQuads();
}
const Index* SofaPhysicsOutputMesh::getQuads()   ///< quads topology (4 indices / quad)
{
    return impl->getQuads();
}
int SofaPhysicsOutputMesh::getQuadsRevision()    ///< changes each time quads data is updated
{
    return impl->getQuadsRevision();
}

////////////////////////////////////////
////////////////////////////////////////
////////////////////////////////////////

using namespace sofa::defaulttype;
using namespace sofa::core::objectmodel;


SofaPhysicsOutputMesh::Impl::Impl()
{
}

SofaPhysicsOutputMesh::Impl::~Impl()
{
}

const char* SofaPhysicsOutputMesh::Impl::getName() ///< (non-unique) name of this object
{
    if (!sObj) return "";
    return sObj->getName().c_str();
}

ID SofaPhysicsOutputMesh::Impl::getID() ///< unique ID of this object
{
    return sObj.get();
}

unsigned int SofaPhysicsOutputMesh::Impl::getNbVertices() ///< number of vertices
{
    if (!sObj) return 0;
    // we cannot use getVertices() method directly as we need the Data revision
    Data<ResizableExtVector<Coord> > * data =
        (!sObj->m_vertPosIdx.getValue().empty()) ?
        &(sObj->m_vertices2) : &(sObj->m_positions);
    return (unsigned int) data->getValue().size();
}
const Real* SofaPhysicsOutputMesh::Impl::getVPositions()  ///< vertices positions (Vec3)
{
    Data<ResizableExtVector<Coord> > * data =
        (!sObj->m_vertPosIdx.getValue().empty()) ?
        &(sObj->m_vertices2) : &(sObj->m_positions);
    return (const Real*) data->getValue().getData();
}
const Real* SofaPhysicsOutputMesh::Impl::getVNormals()    ///< vertices normals   (Vec3)
{
    Data<ResizableExtVector<Deriv> > * data = &(sObj->m_vnormals);
    return (const Real*) data->getValue().getData();
}

const Real* SofaPhysicsOutputMesh::Impl::getVTexCoords()  ///< vertices UVs       (Vec2)
{
    Data<ResizableExtVector<TexCoord> > * data = &(sObj->m_vtexcoords);
    return (const Real*) data->getValue().getData();
}

int SofaPhysicsOutputMesh::Impl::getVerticesRevision()    ///< changes each time vertices data are updated
{
    Data<ResizableExtVector<Coord> > * data =
        (!sObj->m_vertPosIdx.getValue().empty()) ?
        &(sObj->m_vertices2) : &(sObj->m_positions);
    return data->getCounter();
}

unsigned int SofaPhysicsOutputMesh::Impl::getNbLines() ///< number of lines
{
    return 0; // not yet supported
}
const Index* SofaPhysicsOutputMesh::Impl::getLines()   ///< lines topology (2 indices / line)
{
    return NULL;
}
int SofaPhysicsOutputMesh::Impl::getLinesRevision()    ///< changes each time lines data is updated
{
    return 0;
}

unsigned int SofaPhysicsOutputMesh::Impl::getNbTriangles() ///< number of triangles
{
    Data<ResizableExtVector<Triangle> > * data = &(sObj->m_triangles);
    return (unsigned int) data->getValue().size();
}
const Index* SofaPhysicsOutputMesh::Impl::getTriangles()   ///< triangles topology (3 indices / triangle)
{
    Data<ResizableExtVector<Triangle> > * data = &(sObj->m_triangles);
    return (const Index*) data->getValue().getData();
}
int SofaPhysicsOutputMesh::Impl::getTrianglesRevision()    ///< changes each time triangles data is updated
{
    Data<ResizableExtVector<Triangle> > * data = &(sObj->m_triangles);
    return data->getCounter();
}

unsigned int SofaPhysicsOutputMesh::Impl::getNbQuads() ///< number of quads
{
    Data<ResizableExtVector<Quad> > * data = &(sObj->m_quads);
    return (unsigned int) data->getValue().size();
}
const Index* SofaPhysicsOutputMesh::Impl::getQuads()   ///< quads topology (4 indices / quad)
{
    Data<ResizableExtVector<Quad> > * data = &(sObj->m_quads);
    return (const Index*) data->getValue().getData();
}
int SofaPhysicsOutputMesh::Impl::getQuadsRevision()    ///< changes each time quads data is updated
{
    Data<ResizableExtVector<Quad> > * data = &(sObj->m_quads);
    return data->getCounter();
}