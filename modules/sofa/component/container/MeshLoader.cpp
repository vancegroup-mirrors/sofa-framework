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
#include <sofa/component/container/MeshLoader.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/componentmodel/topology/Topology.h>
#include <iostream>

namespace sofa
{

namespace component
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(MeshLoader)

int MeshLoaderClass = core::RegisterObject("Generic Mesh Loader")
.add< MeshLoader >()
;

MeshLoader::MeshLoader()
: filename(initData(&filename,"filename","Filename of the object"))
, triangulate(initData(&triangulate,false,"triangulate","Divide all polygons into triangles"))
, fillMState(initData(&fillMState,true,"fillMState","Must this mesh loader fill the mstate instead of manually or by using the topology"))
{}

void MeshLoader::parse(core::objectmodel::BaseObjectDescription* arg)
{
	this->BaseObject::parse(arg); 
	
	if (filename.getValue() != "") 
		load(filename.getValue().c_str());
}

void MeshLoader::clear()
{
	seqPoints.clear();
	seqEdges.clear(); 
    seqTriangles.clear(); 
	seqQuads.clear();
	seqTetras.clear();
	seqHexas.clear();
}

bool MeshLoader::load(const char* filename)
{
	clear();
	if (!MeshTopologyLoader::load(filename))
    {         
      serr << "Unable to load Mesh \""<<filename << sendl;                      
		return false;
	}

	this->filename.setValue(filename);
	return true;
}

void MeshLoader::addPoint(double px, double py, double pz)
{	
	seqPoints.push_back(helper::make_array((SReal)px, (SReal)py, (SReal)pz));
}

void MeshLoader::addLine( int a, int b )
{
    seqEdges.push_back(Edge(a,b));
}

void MeshLoader::addTriangle( int a, int b, int c )
{
    seqTriangles.push_back( Triangle(a,b,c) );
}

void MeshLoader::addTetra( int a, int b, int c, int d )
{
    seqTetras.push_back( Tetra(a,b,c,d) );
}

void MeshLoader::addQuad(int p1, int p2, int p3, int p4)
{
	if (triangulate.getValue())
	{
		addTriangle(p1,p2,p3);
		addTriangle(p1,p3,p4);
	}
	else
	seqQuads.push_back(Quad(p1,p2,p3,p4));
}

void MeshLoader::addCube(int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8)
{
#ifdef SOFA_NEW_HEXA
	seqHexas.push_back(Hexa(p1,p2,p3,p4,p5,p6,p7,p8));
#else
	seqHexas.push_back(Hexa(p1,p2,p4,p3,p5,p6,p8,p7));
#endif
}

void MeshLoader::getPoints(MeshLoader::SeqPoints& points) const
{
	return points.assign(seqPoints.begin(), seqPoints.end());
}

void MeshLoader::getEdges(MeshLoader::SeqEdges& edges)  const
{
	return edges.assign(seqEdges.begin(), seqEdges.end());
}

void MeshLoader::getTriangles(MeshLoader::SeqTriangles& triangles) const
{
    return triangles.assign(seqTriangles.begin(), seqTriangles.end());
}

void MeshLoader::getQuads(MeshLoader::SeqQuads& quads) const
{
	return quads.assign(seqQuads.begin(), seqQuads.end());
}

void MeshLoader::getTetras(MeshLoader::SeqTetras& tetras) const
{
	return tetras.assign(seqTetras.begin(), seqTetras.end());
}

void  MeshLoader::getHexas(MeshLoader::SeqHexas& hexas) const
{  
	return hexas.assign(seqHexas.begin(), seqHexas.end());
}

int MeshLoader::getNbPoints() const
{
	return seqPoints.size();
}

} // namespace component

} // namespace sofa

