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
#ifndef SOFA_COMPONENT_TOPOLOGY_MESHLOADER_H
#define SOFA_COMPONENT_TOPOLOGY_MESHLOADER_H

#include <string>
#include <sofa/helper/fixed_array.h>
#include <sofa/helper/vector.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/helper/io/MeshTopologyLoader.h>

namespace sofa
{

namespace component
{

class MeshLoader : public virtual core::objectmodel::BaseObject,
					public helper::io::MeshTopologyLoader
{
public:
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
    typedef helper::fixed_array<PointID,4> Tetra;
    typedef helper::fixed_array<PointID,8> Hexa;
        
    typedef helper::vector<Point> SeqPoints;
    typedef helper::vector<Edge> SeqEdges;
    typedef helper::vector<Triangle> SeqTriangles;
    typedef helper::vector<Quad> SeqQuads;
    typedef helper::vector<Tetra> SeqTetras;
    typedef helper::vector<Hexa> SeqHexas;

	MeshLoader();

	virtual ~MeshLoader() {}
	
	virtual void clear();
	
	virtual bool load(const char* filename);
	
	
    void setFilename(std::string f)
	{
		filename.setValue(f);
	}

    const std::string &getFilename() const
	{
		return filename.getValue();
	}

	double getPX(int i) const { return seqPoints.getValue()[i][0]; }
    double getPY(int i) const { return seqPoints.getValue()[i][1]; }
    double getPZ(int i) const { return seqPoints.getValue()[i][2]; }
	
	int getNbPoints() const;
	const SeqPoints& getPoints() const;
	const SeqEdges& getEdges() const;
	const SeqTriangles& getTriangles() const;
	const SeqQuads& getQuads() const;
	const SeqTetras& getTetras() const;
	const SeqHexas& getHexas() const;
	
    void parse(core::objectmodel::BaseObjectDescription* arg);

protected:	
	// helper::io::MeshTopologyLoader API
	void addPoint(double px, double py, double pz);
    void addLine( int a, int b );
	void addTriangle( int a, int b, int c );
    void addTetra( int a, int b, int c, int d );
	void addQuad( int a, int b, int c, int d );
    void addCube( int a, int b, int c, int d, int e, int f, int g, int h );

protected:
	Data<SeqPoints>		seqPoints;
	Data<SeqEdges>		seqEdges;
    Data<SeqTriangles>	seqTriangles;
	Data<SeqQuads>		seqQuads;
	Data<SeqTetras>		seqTetras;
	Data<SeqHexas>		seqHexas;

	Data< std::string > filename;
};

} // namespace component

} // namespace sofa

#endif
