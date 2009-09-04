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
#include <sofa/helper/system/config.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/topology/initTopology.h>
#include <iostream>

namespace sofa
{

namespace component
{


void SOFA_COMPONENT_CONTAINER_API initTopology()
{
    static bool first = true;
    if (first)
    {
//         sout << "Sofa components initialized."<<sendl;

        //std::ofstream ofile("sofa-classes.html");
        //ofile << "<html><body>\n";
        //sofa::core::ObjectFactory::getInstance()->dumpHTML(ofile);
        //ofile << "</body></html>\n";
        first = false;
    }
}

} // namespace component

} // namespace sofa

////////// BEGIN CLASS LIST //////////
SOFA_LINK_CLASS(CenterPointTopologicalMapping)
SOFA_LINK_CLASS(CylinderGridTopology)
SOFA_LINK_CLASS(CubeTopology)
SOFA_LINK_CLASS(ManifoldEdgeSetGeometryAlgorithms)
SOFA_LINK_CLASS(ManifoldEdgeSetTopologyAlgorithms)
SOFA_LINK_CLASS(ManifoldEdgeSetTopologyContainer)
SOFA_LINK_CLASS(ManifoldEdgeSetTopologyModifier)
SOFA_LINK_CLASS(EdgeSetGeometryAlgorithms)
SOFA_LINK_CLASS(EdgeSetTopologyAlgorithms)
SOFA_LINK_CLASS(EdgeSetTopologyContainer)
SOFA_LINK_CLASS(EdgeSetTopologyModifier)
SOFA_LINK_CLASS(GridTopology)
SOFA_LINK_CLASS(Mesh2PointTopologicalMapping)
SOFA_LINK_CLASS(MeshTopology)
SOFA_LINK_CLASS(PointSetGeometryAlgorithms)
SOFA_LINK_CLASS(PointSetTopologyAlgorithms)
SOFA_LINK_CLASS(PointSetTopologyContainer)
SOFA_LINK_CLASS(PointSetTopologyModifier)
SOFA_LINK_CLASS(SimpleTesselatedHexaTopologicalMapping)
SOFA_LINK_CLASS(SimpleTesselatedTetraTopologicalMapping)
SOFA_LINK_CLASS(TriangleSetGeometryAlgorithms)
SOFA_LINK_CLASS(TriangleSetTopologyAlgorithms)
SOFA_LINK_CLASS(TriangleSetTopologyContainer)
SOFA_LINK_CLASS(TriangleSetTopologyModifier)
SOFA_LINK_CLASS(ManifoldTriangleSetTopologyContainer)
SOFA_LINK_CLASS(QuadSetGeometryAlgorithms)
SOFA_LINK_CLASS(QuadSetTopologyAlgorithms)
SOFA_LINK_CLASS(QuadSetTopologyContainer)
SOFA_LINK_CLASS(QuadSetTopologyModifier)
SOFA_LINK_CLASS(HexahedronSetGeometryAlgorithms)
SOFA_LINK_CLASS(HexahedronSetTopologyAlgorithms)
SOFA_LINK_CLASS(HexahedronSetTopologyContainer)
SOFA_LINK_CLASS(HexahedronSetTopologyModifier)
SOFA_LINK_CLASS(TetrahedronSetGeometryAlgorithms)
SOFA_LINK_CLASS(TetrahedronSetTopologyAlgorithms)
SOFA_LINK_CLASS(TetrahedronSetTopologyContainer)
SOFA_LINK_CLASS(TetrahedronSetTopologyModifier)
SOFA_LINK_CLASS(RegularGridTopology)
SOFA_LINK_CLASS(SparseGridTopology)
SOFA_LINK_CLASS(Edge2QuadTopologicalMapping)
SOFA_LINK_CLASS(Triangle2EdgeTopologicalMapping)
SOFA_LINK_CLASS(Quad2TriangleTopologicalMapping)
SOFA_LINK_CLASS(Tetra2TriangleTopologicalMapping)
SOFA_LINK_CLASS(Hexa2QuadTopologicalMapping)

