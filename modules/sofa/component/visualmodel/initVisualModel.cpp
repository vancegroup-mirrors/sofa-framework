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
#include <sofa/component/visualmodel/initVisualModel.h>

namespace sofa
{

namespace component
{


void initVisualModel()
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
SOFA_LINK_CLASS(ClipPlane)
SOFA_LINK_CLASS(DirectionalLight)
SOFA_LINK_CLASS(DrawV)
SOFA_LINK_CLASS(LightManager)
SOFA_LINK_CLASS(OglModel)
SOFA_LINK_CLASS(PointSplatModel)
SOFA_LINK_CLASS(PositionalLight)
SOFA_LINK_CLASS(InteractiveCamera)

#ifdef SOFA_HAVE_GLEW
SOFA_LINK_CLASS(OglRenderingSRGB)
SOFA_LINK_CLASS(OglShader)
SOFA_LINK_CLASS(OglShaderVisualModel)
SOFA_LINK_CLASS(OglShadowShader)
SOFA_LINK_CLASS(OglTexture)
SOFA_LINK_CLASS(OglTexture2D)
SOFA_LINK_CLASS(OglIntVariable)
SOFA_LINK_CLASS(OglInt2Variable)
SOFA_LINK_CLASS(OglInt3Variable)
SOFA_LINK_CLASS(OglInt4Variable)
SOFA_LINK_CLASS(OglFloatVariable)
SOFA_LINK_CLASS(OglFloat2Variable)
SOFA_LINK_CLASS(OglFloat3Variable)
SOFA_LINK_CLASS(OglFloat4Variable)
SOFA_LINK_CLASS(OglIntVectorVariable)
SOFA_LINK_CLASS(OglIntVector2Variable)
SOFA_LINK_CLASS(OglIntVector3Variable)
SOFA_LINK_CLASS(OglIntVector4Variable)
SOFA_LINK_CLASS(OglFloatVectorVariable)
SOFA_LINK_CLASS(OglFloatVector2Variable)
SOFA_LINK_CLASS(OglFloatVector3Variable)
SOFA_LINK_CLASS(OglFloatVector4Variable)
SOFA_LINK_CLASS(OglShaderDefineMacro)
SOFA_LINK_CLASS(OglTetrahedralModel)
SOFA_LINK_CLASS ( OglFloatAttribute );
SOFA_LINK_CLASS ( OglFloat2Attribute );
SOFA_LINK_CLASS ( OglFloat3Attribute );
SOFA_LINK_CLASS ( OglFloat4Attribute );
SOFA_LINK_CLASS ( OglIntAttribute );
SOFA_LINK_CLASS ( OglInt2Attribute );
SOFA_LINK_CLASS ( OglInt3Attribute );
SOFA_LINK_CLASS ( OglInt4Attribute );
SOFA_LINK_CLASS ( OglUIntAttribute );
SOFA_LINK_CLASS ( OglUInt2Attribute );
SOFA_LINK_CLASS ( OglUInt3Attribute );
SOFA_LINK_CLASS ( OglUInt4Attribute );
SOFA_LINK_CLASS(SlicedVolumetricModel)
#endif


