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
#include <sofa/component/collision/initCollision.h>

namespace sofa
{

namespace component
{


void initCollision()
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
SOFA_LINK_CLASS(BarycentricContactMapper)
SOFA_LINK_CLASS(BarycentricPenalityContact)
SOFA_LINK_CLASS(BruteForce)
SOFA_LINK_CLASS(CarvingManager)
SOFA_LINK_CLASS(DefaultContactManager)
SOFA_LINK_CLASS(DefaultPipeline)
SOFA_LINK_CLASS(RuleBasedContactManager)
SOFA_LINK_CLASS(ContinuousIntersection)
SOFA_LINK_CLASS(Cube)
SOFA_LINK_CLASS(DiscreteIntersection)
SOFA_LINK_CLASS(DistanceGridCollisionModel)
SOFA_LINK_CLASS(FrictionContact)
SOFA_LINK_CLASS(ContinuousFrictionContact)
SOFA_LINK_CLASS(Line)
SOFA_LINK_CLASS(LineLocalMinDistanceFilter)
SOFA_LINK_CLASS(LMDNewProximityIntersection)
SOFA_LINK_CLASS(LocalMinDistance)
SOFA_LINK_CLASS(MinProximityIntersection)
SOFA_LINK_CLASS(NewProximityIntersection)
SOFA_LINK_CLASS(Point)
SOFA_LINK_CLASS(PointLocalMinDistanceFilter)
SOFA_LINK_CLASS(Ray)
SOFA_LINK_CLASS(RayContact)
SOFA_LINK_CLASS(RayTraceDetection)
SOFA_LINK_CLASS(Sphere)
SOFA_LINK_CLASS(TetrahedronModel)
SOFA_LINK_CLASS(TreeCollisionGroupManager)
SOFA_LINK_CLASS(Triangle)
SOFA_LINK_CLASS(TriangleLocalMinDistanceFilter)
SOFA_LINK_CLASS(SphereTreeModel)

#ifdef SOFA_HAVE_EIGEN2
SOFA_LINK_CLASS(BarycentricDistanceLMConstraintContact)
#endif

