/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGY_H
#define SOFA_COMPONENT_TOPOLOGY_TRIANGLESETTOPOLOGY_H

#include <sofa/component/topology/EdgeSetTopology.h>
#include <vector>
#include <map>

#include <sofa/defaulttype/Vec.h> // typing "Vec"

namespace sofa
{

namespace component
{

namespace topology
{
using namespace sofa::defaulttype; // typing "Vec"

/// defining Triangles as 3 DOFs indices
typedef helper::fixed_array<unsigned int,3> Triangle;
/// defining TriangleEdges as 3 Edge indices
typedef helper::fixed_array<unsigned int,3> TriangleEdges;

/////////////////////////////////////////////////////////
/// TopologyChange subclasses
/////////////////////////////////////////////////////////



/** indicates that some triangles were added */
class TrianglesAdded : public core::componentmodel::topology::TopologyChange
{

public:
    unsigned int nTriangles;

    std::vector< Triangle > triangleArray;

    std::vector< unsigned int > triangleIndexArray;

    std::vector< std::vector< unsigned int > > ancestorsList;

    std::vector< std::vector< double > > coefs;

    TrianglesAdded(const unsigned int nT,
            const std::vector< Triangle >& _triangleArray = (const std::vector< Triangle >)0,
            const std::vector< unsigned int >& trianglesIndex = (const std::vector< unsigned int >)0,
            const std::vector< std::vector< unsigned int > >& ancestors = (const std::vector< std::vector< unsigned int > >)0,
            const std::vector< std::vector< double > >& baryCoefs = (const std::vector< std::vector< double > >)0)
        : core::componentmodel::topology::TopologyChange(core::componentmodel::topology::TRIANGLESADDED), nTriangles(nT), triangleArray(_triangleArray), triangleIndexArray(trianglesIndex),ancestorsList(ancestors), coefs(baryCoefs)
    {   }

    unsigned int getNbAddedTriangles() const
    {
        return nTriangles;
    }

};



/** indicates that some triangles are about to be removed */
class TrianglesRemoved : public core::componentmodel::topology::TopologyChange
{

public:
    std::vector<unsigned int> removedTrianglesArray;

public:
    TrianglesRemoved(const std::vector<unsigned int> _tArray) : core::componentmodel::topology::TopologyChange(core::componentmodel::topology::TRIANGLESREMOVED), removedTrianglesArray(_tArray)
    {
    }

    const std::vector<unsigned int> &getArray() const
    {
        return removedTrianglesArray;
    }
    unsigned int getNbRemovedTriangles() const
    {
        return removedTrianglesArray.size();
    }

};



/////////////////////////////////////////////////////////
/// TriangleSetTopology objects
/////////////////////////////////////////////////////////


/** Object that stores a set of triangles and provides access
to each triangle and its edges and vertices */
class TriangleSetTopologyContainer : public EdgeSetTopologyContainer
{
private:
    /** \brief Creates the array of edge indices for each triangle
    *
    * This function is only called if the TriangleEdge array is required.
    * m_triangleEdge[i] contains the 3 indices of the 3 edges opposite to the ith vertex
    */
    void createTriangleEdgeArray();
    /** \brief Creates the Triangle Vertex Shell Array
    *
    * This function is only called if the TriangleVertexShell array is required.
    * m_triangleVertexShell[i] contains the indices of all triangles adjacent to the ith vertex
    */
    void createTriangleVertexShellArray();

    /** \brief Creates the Triangle Edge Shell Array
    *
    * This function is only called if the TriangleVertexShell array is required.
    * m_triangleEdgeShell[i] contains the indices of all triangles adjacent to the ith edge
    */
    void createTriangleEdgeShellArray();
protected:
    /// provides the set of triangles
    std::vector<Triangle> m_triangle;
    /// provides the 3 edges in each triangle
    std::vector<TriangleEdges> m_triangleEdge;
    /// for each vertex provides the set of triangles adjacent to that vertex
    std::vector< std::vector< unsigned int > > m_triangleVertexShell;
    /// for each edge provides the set of triangles adjacent to that edge
    std::vector< std::vector< unsigned int > > m_triangleEdgeShell;


    /** \brief Creates the TriangleSet array.
     *
     * This function is only called by derived classes to create a list of triangles from a set of tetrahedra for instance
    */
    virtual void createTriangleSetArray() {}

    /** \brief Creates the EdgeSet array.
     *
     * Create the set of edges when needed.
    */
    virtual void createEdgeSetArray() {createTriangleEdgeArray();}

public:
    /** \brief Returns the Triangle array.
     *
     */
    const std::vector<Triangle> &getTriangleArray();

    /** \brief Returns the ith Triangle.
     *
     */
    const Triangle &getTriangle(const unsigned int i);

    /** \brief Returns the number of triangles in this topology.
     *
     */
    unsigned int getNumberOfTriangles() ;

    /** \brief Returns the Triangle Vertex Shells array.
     *
     */
    const std::vector< std::vector<unsigned int> > &getTriangleVertexShellArray() ;

    /** \brief Returns the set of triangles adjacent to a given vertex.
     *
     */
    const std::vector< unsigned int > &getTriangleVertexShell(const unsigned int i) ;


    /** \brief Returns the TriangleEdges array (ie provide the 3 edge indices for each triangle)
     *
     */
    const std::vector< TriangleEdges > &getTriangleEdgeArray() ;

    /** \brief Returns the 3 edges adjacent to a given triangle.
     *
     */
    const TriangleEdges &getTriangleEdge(const unsigned int i) ;

    /** \brief Returns the Triangle Edge Shells array (ie provides the triangles adjacent to each edge)
     *
     */
    const std::vector< std::vector<unsigned int> > &getTriangleEdgeShellArray() ;

    /** \brief Returns the set of triangles adjacent to a given edge.
     *
     */
    const std::vector< unsigned int > &getTriangleEdgeShell(const unsigned int i) ;


    /** Returns the indices of a triangle given three vertex indices : returns -1 if none */
    int getTriangleIndex(const unsigned int v1, const unsigned int v2, const unsigned int v3);


    /** returns the index (either 0, 1 ,2) of the vertex whose global index is vertexIndex. Returns -1 if none */
    int getVertexIndexInTriangle(Triangle &t,unsigned int vertexIndex) const;

    /** returns the index (either 0, 1 ,2) of the edge whose global index is edgeIndex. Returns -1 if none */
    int getEdgeIndexInTriangle(TriangleEdges &t,unsigned int edheIndex) const;

    /** \brief Checks if the Triangle Set Topology is coherent
     *
     * Check if the Edge and the Edhe Shell arrays are coherent
     */
    virtual bool checkTopology() const;

    TriangleSetTopologyContainer(core::componentmodel::topology::BaseTopology *top,
            const std::vector< unsigned int > &DOFIndex = (const std::vector< unsigned int >)0,
            const std::vector< Triangle >         &triangles    = (const std::vector< Triangle >)        0 );

    template< typename DataTypes >
    friend class TriangleSetTopologyModifier;
private:
    /** \brief Returns a non-const triangle vertex shell given a vertex index for subsequent modification
     *
     */
    std::vector< unsigned int > &getTriangleVertexShellForModification(const unsigned int vertexIndex);
    /** \brief Returns a non-const triangle edge shell given the index of an edge for subsequent modification
      *
      */
    std::vector< unsigned int > &getTriangleEdgeShellForModification(const unsigned int edgeIndex);


};


template <class DataTypes>
class TriangleSetTopologyLoader;

/**
 * A class that modifies the topology by adding and removing triangles
 */
template<class DataTypes>
class TriangleSetTopologyModifier : public EdgeSetTopologyModifier <DataTypes>
{

public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;

    TriangleSetTopologyModifier(core::componentmodel::topology::BaseTopology *top) : EdgeSetTopologyModifier<DataTypes>(top)
    {
    }
    /** \brief Build a triangle set topology from a file : also modifies the MechanicalObject
     *
     */
    virtual bool load(const char *filename);


    /** \brief Sends a message to warn that some triangles were added in this topology.
     *
     * \sa addTrianglesProcess
     */
    void addTrianglesWarning(const unsigned int nTriangles,
            const std::vector< Triangle >& trianglesList,
            const std::vector< unsigned int >& trianglesIndexList,
            const std::vector< std::vector< unsigned int > > & ancestors= (const std::vector< std::vector<unsigned int > >) 0 ,
            const std::vector< std::vector< double > >& baryCoefs= (const std::vector< std::vector< double > >)0) ;



    /** \brief Actually Add some triangles to this topology.
     *
     * \sa addTrianglesWarning
     */
    virtual void addTrianglesProcess(const std::vector< Triangle > &triangles);

    /** \brief Sends a message to warn that some triangles are about to be deleted.
     *
     * \sa removeTrianglesProcess
     *
     * Important : parameter indices is not const because it is actually sorted from the highest index to the lowest one.
     */
    virtual void removeTrianglesWarning( std::vector<unsigned int> &triangles);

    /** \brief Remove a subset of  triangles. Eventually remove isolated edges and vertices
     *
     * Important : some structures might need to be warned BEFORE the points are actually deleted, so always use method removeEdgesWarning before calling removeEdgesProcess.
     * \sa removeTrianglesWarning
     *
     * @param removeIsolatedItems if true isolated vertices are also removed
     */
    virtual void removeTrianglesProcess( const std::vector<unsigned int> &indices,const bool removeIsolatedItems=false);

    /** \brief Add some edges to this topology.
    *
    * \sa addEdgesWarning
    */
    virtual void addEdgesProcess(const std::vector< Edge > &edges);


    /** \brief Remove a subset of edges
    *
    * Important : some structures might need to be warned BEFORE the points are actually deleted, so always use method removeEdgesWarning before calling removeEdgesProcess.
    * \sa removeEdgesWarning
    *
    * @param removeIsolatedItems if true isolated vertices are also removed
    * Important : parameter indices is not const because it is actually sorted from the highest index to the lowest one.
    */
    virtual void removeEdgesProcess( const std::vector<unsigned int> &indices,const bool removeIsolatedItems=false);



    /** \brief Add some points to this topology.
     *
     * Use a list of ancestors to create the new points.
     * Last parameter baryCoefs defines the coefficient used for the creation of the new points.
     * Default value for these coefficient (when none is defined) is 1/n with n being the number of ancestors
     * for the point being created.
     *
     * \sa addPointsWarning
     */
    virtual void addPointsProcess(const unsigned int nPoints,
            const std::vector< std::vector< unsigned int > >& ancestors = (const std::vector< std::vector< unsigned int > >)0,
            const std::vector< std::vector< double > >& baryCoefs = (const std::vector< std::vector< double > >)0 );



    /** \brief Remove a subset of points
     *
     * Elements corresponding to these points are removed from the mechanical object's state vectors.
     *
     * Important : some structures might need to be warned BEFORE the points are actually deleted, so always use method removePointsWarning before calling removePointsProcess.
     * \sa removePointsWarning
     */
    virtual void removePointsProcess(std::vector<unsigned int> &indices);



    /** \brief Reorder this topology.
     *
     * \see MechanicalObject::renumberValues
     */
    virtual void renumberPointsProcess( const std::vector<unsigned int> &index );


protected:
    void addTriangle(Triangle e);

public:
    //template <class DataTypes>
    friend class TriangleSetTopologyLoader<DataTypes>;

};



/**
 * A class that performs topology algorithms on an TriangleSet.
 */
template < class DataTypes >
class TriangleSetTopologyAlgorithms : public PointSetTopologyAlgorithms<DataTypes>
{

public:

    TriangleSetTopologyAlgorithms(sofa::core::componentmodel::topology::BaseTopology *top) : PointSetTopologyAlgorithms<DataTypes>(top)
    {
    }

    /** \brief Remove a set  of triangles
    @param triangles an array of triangle indices to be removed (note that the array is not const since it needs to be sorted)
    *
    */
    virtual void removeTriangles(std::vector< unsigned int >& triangles);

    // Prepares the incision along the list of points (ind_edge,coord) intersected by the vector from point a to point b
    // and the triangular mesh
    double Prepare_InciseAlongPointsList(const Vec<3,double>& a, const Vec<3,double>& b, const unsigned int ind_ta, const unsigned int ind_tb, unsigned int &new_ind_ta, unsigned int &newind_tb);


    // Incises along the list of points (ind_edge,coord) intersected by the vector from point a to point b
    // and the triangular mesh
    void InciseAlongPointsList(const Vec<3,double>& a, const Vec<3,double>& b, const unsigned int ind_ta, const unsigned int ind_tb);

    // Removes triangles along the list of points (ind_edge,coord) intersected by the vector from point a to point b
    // and the triangular mesh
    void RemoveAlongTrianglesList(const Vec<3,double>& a, const Vec<3,double>& b, const unsigned int ind_ta, const unsigned int ind_tb);

    // Incises along the list of points (ind_edge,coord) intersected by the sequence of input segments (list of input points) and the triangular mesh
    void InciseAlongLinesList(const std::vector< Vec<3,double> >& input_points, const std::vector< unsigned int > &input_triangles);

};

/**
 * A class that provides geometry information on an TriangleSet.
 */
template < class DataTypes >
class TriangleSetGeometryAlgorithms : public EdgeSetGeometryAlgorithms<DataTypes>
{
public:
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::Coord Coord;


    TriangleSetGeometryAlgorithms(sofa::core::componentmodel::topology::BaseTopology *top) : EdgeSetGeometryAlgorithms<DataTypes>(top)
    {
    }
    /// computes the area of triangle no i and returns it
    Real computeTriangleArea(const unsigned int i) const;
    /// computes the triangle area of all triangles are store in the array interface
    void computeTriangleArea( BasicArrayInterface<Real> &ai) const;
    /// computes the initial area  of triangle no i and returns it
    Real computeRestTriangleArea(const unsigned int i) const;

    // barycentric coefficients of point p in triangle (a,b,c) indexed by ind_t
    std::vector< double > computeTriangleBarycoefs( const Vec<3,double> &p, unsigned int ind_t);

    // Computes the point defined by 2 indices of vertex and 1 barycentric coordinate
    Vec<3,double> computeBaryEdgePoint(std::vector< unsigned int>& indices, const double &coord_p);

    // Computes the normal vector of a triangle indexed by ind_t (not normed)
    Vec<3,double> computeTriangleNormal(const unsigned int ind_t);

    // Computes the opposite point to ind_p
    Vec<3,double> getOppositePoint(unsigned int ind_p, std::vector< unsigned int>& indices, const double &coord_p);

    // Test if a triangle indexed by ind_t (and incident to the vertex indexed by ind_p) is included or not in the plane defined by (ind_p, plane_vect)
    bool is_triangle_in_plane(const unsigned int ind_t, const unsigned int ind_p, const Vec<3,double>& plane_vect);


    // Prepares the duplication of a vertex
    void Prepare_VertexDuplication(const unsigned int ind_p, const unsigned int ind_t_from, const unsigned int ind_t_to,
            const std::vector< unsigned int>& indices_from, const double &coord_from, const std::vector< unsigned int>& indices_to, const double &coord_to,
            std::vector< unsigned int > &triangles_list_1, std::vector< unsigned int > &triangles_list_2);



    // Computes the intersection of the vector from point a to point b and the triangle indexed by t
    bool computeSegmentTriangleIntersection(bool is_entered, const Vec<3,double>& a, const Vec<3,double>& b, const unsigned int ind_t,
            std::vector<unsigned int> &indices,
            double &baryCoef, double& coord_kmin);

    // Computes the list of points (ind_edge,coord) intersected by the segment from point a to point b
    // and the triangular mesh
    bool computeIntersectedPointsList(const Vec<3,double>& a, const Vec<3,double>& b, const unsigned int ind_ta, const unsigned int ind_tb,
            std::vector< unsigned int > &triangles_list, std::vector< std::vector< unsigned int> > &indices_list, std::vector< double >& coords_list);
};


/** Describes a topological object that only consists as a set of points :
it is a base class for all topological objects */
template<class DataTypes>
class TriangleSetTopology : public EdgeSetTopology <DataTypes>
{

public:
    TriangleSetTopology(component::MechanicalObject<DataTypes> *obj);


    virtual void init();
    /** \brief Returns the TriangleSetTopologyContainer object of this TriangleSetTopology.
     */
    TriangleSetTopologyContainer *getTriangleSetTopologyContainer() const
    {
        return (TriangleSetTopologyContainer *)this->m_topologyContainer;
    }
    /** \brief Returns the TriangleSetTopologyAlgorithms object of this TriangleSetTopology.
     */
    TriangleSetTopologyAlgorithms<DataTypes> *getTriangleSetTopologyAlgorithms() const
    {
        return (TriangleSetTopologyAlgorithms<DataTypes> *)this->m_topologyAlgorithms;
    }
    /** \brief Returns the TriangleSetTopologyAlgorithms object of this TriangleSetTopology.
     */
    TriangleSetGeometryAlgorithms<DataTypes> *getTriangleSetGeometryAlgorithms() const
    {
        return (TriangleSetGeometryAlgorithms<DataTypes> *)this->m_geometryAlgorithms;
    }

};

} // namespace topology

} // namespace component

} // namespace sofa

#endif
