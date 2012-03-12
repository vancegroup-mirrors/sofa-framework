/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
* Authors: The SOFA Team (see Authors.txt)                                    *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
/*
 * CompositingVisualLoop.cpp
 *
 *  Created on: 16 janv. 2012
 *      Author: Jeremy Ringard
 */
#include <sofa/component/visualmodel/CompositingVisualLoop.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/common/VisualVisitor.h>
#include <sofa/simulation/common/UpdateContextVisitor.h>
#include <sofa/simulation/common/UpdateMappingVisitor.h>
#include <sofa/simulation/common/UpdateMappingEndEvent.h>
#include <sofa/simulation/common/PropagateEventVisitor.h>


#include <sofa/helper/AdvancedTimer.h>

namespace sofa
{
namespace component
{
namespace visualmodel
{

SOFA_DECL_CLASS(CompositingVisualLoop);

int CompositingVisualLoopClass = core::RegisterObject("Visual loop enabling multipass rendering. Needs multiple fbo data and a compositing shader")
        .add< CompositingVisualLoop >()
        ;

CompositingVisualLoop::CompositingVisualLoop(simulation::Node* _gnode)
    : simulation::DefaultVisualManagerLoop(_gnode),
      vertFilename(initData(&vertFilename, (std::string) "shaders/compositing.vert", "vertFilename", "Set the vertex shader filename to load")),
      fragFilename(initData(&fragFilename, (std::string) "shaders/compositing.frag", "fragFilename", "Set the fragment shader filename to load"))
{
    //assert(gRoot);
}

CompositingVisualLoop::~CompositingVisualLoop()
{}

void CompositingVisualLoop::initVisual()
{}

void CompositingVisualLoop::init()
{
    if (!gRoot)
        gRoot = dynamic_cast<simulation::Node*>(this->getContext());
}

//should not be called if scene file is well formed
void CompositingVisualLoop::defaultRendering(sofa::core::visual::VisualParams* vparams)
{
    vparams->pass() = sofa::core::visual::VisualParams::Std;
    VisualDrawVisitor act ( vparams );
    gRoot->execute ( &act );
    vparams->pass() = sofa::core::visual::VisualParams::Transparent;
    VisualDrawVisitor act2 ( vparams );
    gRoot->execute ( &act2 );
}

void CompositingVisualLoop::drawStep(sofa::core::visual::VisualParams* vparams)
{
    if ( !gRoot ) return;

    //should not happen: the compositing loop relies on one or more rendered passes done by the VisualManagerPass component
    if (gRoot->visualManager.empty())
    {
        serr << "CompositingVisualLoop: no VisualManagerPass found. Disable multipass rendering." << sendl;
        defaultRendering(vparams);
    }

    //rendering sequence: call each VisualManagerPass elements, then composite the frames
    else
    {
        Node::Sequence<core::visual::VisualManager>::iterator begin = gRoot->visualManager.begin(), end = gRoot->visualManager.end(), it;
        //preDraw sequence
        for (it = begin; it != end; ++it)
            (*it)->preDrawScene(vparams);
        //Draw sequence
        bool rendered = false; // true if a manager did the rendering
        for (it = begin; it != end; ++it)
            if ((*it)->drawScene(vparams))	{ rendered = true;	break;	}

        if (!rendered) // do the rendering
        {
            std::cerr << "VisualLoop error: no visualManager rendered the scene. Please make sure the final visualManager(Secondary)Pass has a renderToScreen=\"true\" attribute" << std::endl;
        }
        //postDraw sequence
        Node::Sequence<core::visual::VisualManager>::reverse_iterator rbegin = gRoot->visualManager.rbegin(), rend = gRoot->visualManager.rend(), rit;
        for (rit = rbegin; rit != rend; ++rit)
            (*rit)->postDrawScene(vparams);
    }
}

//render a fullscreen quad
void CompositingVisualLoop::traceFullScreenQuad()
{
    float vxmax, vymax, vzmax ;
    float vxmin, vymin, vzmin ;
    float txmax,tymax,tzmax;
    float txmin,tymin,tzmin;

    txmin = tymin = tzmin = 0.0;
    vxmin = vymin = vzmin = -1.0;
    vxmax = vymax = vzmax = txmax = tymax = tzmax = 1.0;

    glBegin(GL_QUADS);
    {
        glTexCoord3f(txmin,tymax,0.0); glVertex3f(vxmin,vymax,0.0);
        glTexCoord3f(txmax,tymax,0.0); glVertex3f(vxmax,vymax,0.0);
        glTexCoord3f(txmax,tymin,0.0); glVertex3f(vxmax,vymin,0.0);
        glTexCoord3f(txmin,tymin,0.0); glVertex3f(vxmin,vymin,0.0);
    }
    glEnd();
}

} // namespace visualmodel
} // namespace component
} //sofa