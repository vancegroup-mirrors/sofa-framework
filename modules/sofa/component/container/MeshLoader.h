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
#ifndef SOFA_COMPONENT_MESHLOADER_H
#define SOFA_COMPONENT_MESHLOADER_H

#include <string>
#include <sofa/helper/fixed_array.h>
#include <sofa/helper/vector.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/helper/io/MeshTopologyLoader.h>
#include <sofa/core/topology/BaseMeshTopology.h> //Need to include this file in order to have the definition of SOFA_NEW_HEXA !
#include <sofa/component/component.h>

namespace sofa
{

namespace component
{

namespace container
{

class SOFA_COMPONENT_CONTAINER_API MeshLoader : public virtual core::objectmodel::BaseObject,
					public helper::io::MeshTopologyLoader
{
public:
	SOFA_CLASS(MeshLoader,core::objectmodel::BaseObject);

	typedef unsigned int index_type;
    typedef index_type PointID;
    typedef index_type EdgeID;
    typedef index_type TriangleID;
    typedef index_type QuadID;
    typedef index_type TetraID;
    typedef index_type HexaID;

    typedef helper::fixed_array<SReal,3> Point;
    typedef helper::fixed_array<PointID,2> Edge;
    typedef helper::fixed_array<PointID,3> Triangle;
    typedef helper::fixed_array<PointID,4> Quad;
    typedef helper::fixed_array<PointID,4> Tetrahedron;
    typedef helper::fixed_array<PointID,8> Hexahedron;

    typedef helper::vector<Point> SeqPoints;
    typedef helper::vector<Edge> SeqEdges;
    typedef helper::vector<Triangle> SeqTriangles;
    typedef helper::vector<Quad> SeqQuads;
    typedef helper::vector<Tetrahedron> SeqTetrahedra;
    typedef helper::vector<Hexahedron> SeqHexahedra;

	MeshLoader();

	virtual ~MeshLoader() {}

	virtual void clear();

        virtual void init();

	virtual bool load(const char* filename);


    void setFilename(std::string f)
	{
		filename.setValue(f);
	}

    const std::string &getFilename() const
	{
		return filename.getValue();
	}

	double getPX(int i) const { return seqPoints[i][0]; }
        double getPY(int i) const { return seqPoints[i][1]; }
        double getPZ(int i) const { return seqPoints[i][2]; }

	virtual int getNbPoints() const;
	virtual void getPoints(SeqPoints& ) const;
	virtual void getEdges(SeqEdges& ) const;
	virtual void getTriangles(SeqTriangles& ) const;
	virtual void getQuads(SeqQuads& ) const;
	virtual void getTetrahedra(SeqTetrahedra& ) const;
	virtual void getHexahedra(SeqHexahedra& ) const;
	
	bool getFillMState( ) const { return fillMState.getValue(); }


protected:
	// helper::io::MeshTopologyLoader API
	void addPoint(double px, double py, double pz);
        void addLine( int a, int b );
	void addTriangle( int a, int b, int c );
        void addTetra( int a, int b, int c, int d );
	void addQuad( int a, int b, int c, int d );
        void addCube( int a, int b, int c, int d, int e, int f, int g, int h );

protected:
	SeqPoints		seqPoints;
	SeqEdges		seqEdges;
	SeqTriangles            seqTriangles;
	SeqQuads		seqQuads;
	SeqTetrahedra		seqTetrahedra;
	SeqHexahedra		seqHexahedra;

	sofa::core::objectmodel::DataFileName filename;
	Data< bool > triangulate;
	Data< bool > fillMState; ///< Must this mesh loader fill the mstate instead of manually or by using the topology 
        Data< helper::vector<sofa::defaulttype::Vector3> > vertices; 
        Data< helper::vector<sofa::defaulttype::Vector3> > texCoords; // for the moment, we suppose that texCoords is order 2 (2 texCoords for a vertex) 
        Data< helper::vector<sofa::defaulttype::Vector3> > normals; 
        Data< helper::vector< helper::vector < helper::vector <int> > > > facets; 
        
	helper::vector<sofa::defaulttype::Vector3> computeNormals();
};

}

} // namespace component

} // namespace sofa

#endif
