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
#include <sofa/component/constraint/initConstraint.h>

namespace sofa
{

namespace component
{


void initConstraint()
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
SOFA_LINK_CLASS(AttachConstraint)
SOFA_LINK_CLASS(UncoupledConstraintCorrection)
SOFA_LINK_CLASS(FixedConstraint)
SOFA_LINK_CLASS(FixedPlaneConstraint)
SOFA_LINK_CLASS(HermiteSplineConstraint)
SOFA_LINK_CLASS(LinearSolverConstraintCorrection)
SOFA_LINK_CLASS(OscillatorConstraint)
SOFA_LINK_CLASS(ParabolicConstraint)
SOFA_LINK_CLASS(PrecomputedConstraintCorrection)
SOFA_LINK_CLASS(LinearMovementConstraint)
SOFA_LINK_CLASS(FixedRotationConstraint)
SOFA_LINK_CLASS(LinearVelocityConstraint)
SOFA_LINK_CLASS(LCPConstraintSolver)
#ifdef SOFA_HAVE_EIGEN2
SOFA_LINK_CLASS(LMConstraintSolver)
SOFA_LINK_CLASS(DistanceLMConstraint)
SOFA_LINK_CLASS(FixedLMConstraint)
SOFA_LINK_CLASS(RotationLMConstraint)
#endif

