/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#ifndef SOFA_COMPONENT_TOPOLOGY_EDGESETGEOMETRYALGORITHMS_INL
#define SOFA_COMPONENT_TOPOLOGY_EDGESETGEOMETRYALGORITHMS_INL

#include <sofa/component/topology/EdgeSetGeometryAlgorithms.h>

#include <sofa/component/topology/PointSetGeometryAlgorithms.inl>

namespace sofa
{

namespace component
{

namespace topology
{
using namespace sofa::defaulttype;

template< class DataTypes>
typename DataTypes::Real EdgeSetGeometryAlgorithms< DataTypes >::computeEdgeLength( const EdgeID i) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const VecCoord& p = *(this->object->getX());
    const Real length = (p[e[0]]-p[e[1]]).norm();
    return length;
}

template< class DataTypes>
typename DataTypes::Real EdgeSetGeometryAlgorithms< DataTypes >::computeRestEdgeLength( const EdgeID i) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const VecCoord& p = *(this->object->getX0());
    const Real length = (p[e[0]]-p[e[1]]).norm();
    return length;
}

template< class DataTypes>
typename DataTypes::Real EdgeSetGeometryAlgorithms< DataTypes >::computeRestSquareEdgeLength( const EdgeID i) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const VecCoord& p = *(this->object->getX0());
    const Real length = (p[e[0]]-p[e[1]]).norm2();
    return length;
}

/// computes the edge length of all edges are store in the array interface
template<class DataTypes>
void EdgeSetGeometryAlgorithms<DataTypes>::computeEdgeLength( BasicArrayInterface<Real> &ai) const
{
    const sofa::helper::vector<Edge> &ea = this->m_topology->getEdges();
    const typename DataTypes::VecCoord& p = *(this->object->getX());

    for (unsigned int i=0; i<ea.size(); ++i)
    {
        const Edge &e = ea[i];
        ai[i] = (p[e[0]]-p[e[1]]).norm();
    }
}

template<class DataTypes>
void EdgeSetGeometryAlgorithms<DataTypes>::computeEdgeAABB(const EdgeID i, Coord& minCoord, Coord& maxCoord) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const typename DataTypes::VecCoord& p = *(this->object->getX());

    const Coord sum(p[e[0]] + p[e[1]]);
    Coord dif(p[e[1]] - p[e[0]]);

    if(dif[0] < 0) dif[0] = p[e[0]][0] - p[e[1]][0];
    if(dif[1] < 0) dif[1] = p[e[0]][1] - p[e[1]][1];
    if(dif[2] < 0) dif[2] = p[e[0]][2] - p[e[1]][2];

    minCoord = (sum-dif) * (Real) 0.5;
    maxCoord = (sum+dif) * (Real) 0.5;
}

template<class DataTypes>
void EdgeSetGeometryAlgorithms<DataTypes>::getEdgeVertexCoordinates(const EdgeID i, Coord pnt[2]) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const typename DataTypes::VecCoord& p = *(this->object->getX());

    pnt[0] = p[e[0]];
    pnt[1] = p[e[1]];
}

template<class DataTypes>
void EdgeSetGeometryAlgorithms<DataTypes>::getRestEdgeVertexCoordinates(const EdgeID i, Coord pnt[2]) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const typename DataTypes::VecCoord& p = *(this->object->getX0());

    pnt[0] = p[e[0]];
    pnt[1] = p[e[1]];
}

template<class DataTypes>
typename DataTypes::Coord EdgeSetGeometryAlgorithms<DataTypes>::computeEdgeCenter(const EdgeID i) const
{
    const Edge &e = this->m_topology->getEdge(i);
    const typename DataTypes::VecCoord& p = *(this->object->getX());

    return (p[e[0]] + p[e[1]]) * (Real) 0.5;
}

/// Write the current mesh into a msh file
template <typename DataTypes>
void EdgeSetGeometryAlgorithms<DataTypes>::writeMSHfile(const char *filename) const
{
    std::ofstream myfile;
    myfile.open (filename);

    const typename DataTypes::VecCoord& vect_c = *(this->object->getX());

    const unsigned int numVertices = vect_c.size();

    myfile << "$NOD\n";
    myfile << numVertices <<"\n";

    for (unsigned int i=0; i<numVertices; ++i)
    {
        double x = (double) vect_c[i][0];
        double y = (double) vect_c[i][1];
        double z = (double) vect_c[i][2];

        myfile << i+1 << " " << x << " " << y << " " << z <<"\n";
    }

    myfile << "$ENDNOD\n";
    myfile << "$ELM\n";

    const sofa::helper::vector<Edge> &edge = this->m_topology->getEdges();

    myfile << edge.size() <<"\n";

    for (unsigned int i=0; i<edge.size(); ++i)
    {
        myfile << i+1 << " 1 1 1 2 " << edge[i][0]+1 << " " << edge[i][1]+1 <<"\n";
    }

    myfile << "$ENDELM\n";

    myfile.close();
}
} // namespace topology

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENTS_EDGESETGEOMETRYALGORITHMS_INL
