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
#include <sofa/component/MeshLoader.h>
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
: seqPoints(initData(&seqPoints,"points","List of points"))
, seqEdges(initData(&seqEdges,"lines","List of line indices"))
, seqTriangles(initData(&seqTriangles,"triangles","List of triangle indices"))
, seqQuads(initData(&seqQuads,"quads","List of quad indices"))
, seqTetras(initData(&seqTetras,"tetras","List of tetra indices"))
, seqHexas(initData(&seqHexas,"hexas","List of hexa indices"))
, filename(initData(&filename,"filename","Filename of the object"))
{
	this->getContext()->addObject(this);
}

void MeshLoader::parse(core::objectmodel::BaseObjectDescription* arg)
{
	this->BaseObject::parse(arg); 
	
	if (filename.getValue() != "") 
		load(filename.getValue().c_str());
}

void MeshLoader::clear()
{
	seqPoints.beginEdit()->clear();
	seqPoints.endEdit();

	seqEdges.beginEdit()->clear(); 
	seqEdges.endEdit();

    seqTriangles.beginEdit()->clear(); 
	seqTriangles.endEdit();

	seqQuads.beginEdit()->clear();
	seqQuads.endEdit();

	seqTetras.beginEdit()->clear();
	seqTetras.endEdit();

	seqHexas.beginEdit()->clear();
	seqHexas.endEdit();
}

bool MeshLoader::load(const char* filename)
{
	clear();
	if (!MeshTopologyLoader::load(filename))
    {         
		logWarning(std::string("Unable to load Mesh \"") + filename );                      
		return false;
	}

	this->filename.setValue(filename);
	return true;
}

void MeshLoader::addPoint(double px, double py, double pz)
{	
	seqPoints.beginEdit()->push_back(helper::make_array((SReal)px, (SReal)py, (SReal)pz));
	seqPoints.endEdit();
}

void MeshLoader::addLine( int a, int b )
{
    seqEdges.beginEdit()->push_back(Edge(a,b));
    seqEdges.endEdit();
}

void MeshLoader::addTriangle( int a, int b, int c )
{
    seqTriangles.beginEdit()->push_back( Triangle(a,b,c) );
    seqTriangles.endEdit();
}

void MeshLoader::addTetra( int a, int b, int c, int d )
{
    seqTetras.beginEdit()->push_back( Tetra(a,b,c,d) );
    seqTetras.endEdit();
}

void MeshLoader::addQuad(int p1, int p2, int p3, int p4)
{
	seqQuads.beginEdit()->push_back(Quad(p1,p2,p3,p4));
	seqQuads.endEdit();
}

void MeshLoader::addCube(int p1, int p2, int p3, int p4, int p5, int p6, int p7, int p8)
{
#ifdef SOFA_NEW_HEXA
	seqHexas.beginEdit()->push_back(Hexa(p1,p2,p3,p4,p5,p6,p7,p8));
#else
	seqHexas.beginEdit()->push_back(Hexa(p1,p2,p4,p3,p5,p6,p8,p7));
#endif
	seqHexas.endEdit();
}

const MeshLoader::SeqPoints& MeshLoader::getPoints() const
{
	return seqPoints.getValue();
}

const MeshLoader::SeqEdges& MeshLoader::getEdges()  const
{
	return seqEdges.getValue();
}

const MeshLoader::SeqTriangles& MeshLoader::getTriangles() const
{
    return seqTriangles.getValue();
}

const MeshLoader::SeqQuads& MeshLoader::getQuads() const
{
	return seqQuads.getValue();
}

const MeshLoader::SeqTetras& MeshLoader::getTetras() const
{
	return seqTetras.getValue();
}

const MeshLoader::SeqHexas& MeshLoader::getHexas() const
{  
	return seqHexas.getValue();
}

int MeshLoader::getNbPoints() const
{
	return seqPoints.getValue().size();
}

} // namespace component

} // namespace sofa

