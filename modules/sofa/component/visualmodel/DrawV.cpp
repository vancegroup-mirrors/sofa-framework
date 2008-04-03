#include <sofa/component/visualmodel/DrawV.h>
#include <sofa/helper/gl/RAII.h>
#include <sofa/helper/vector.h>
#include <sofa/core/ObjectFactory.h>
#include <sstream>

namespace sofa
{

namespace component
{

namespace visualmodel
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(DrawV)

int DrawVClass = core::RegisterObject("Visual model displaying velocity vectors for OpenGL display")
.add< DrawV >()
;

DrawV::DrawV()
: castShadow( initData(&castShadow, false, "castShadow", "True if object cast shadows") )
, useAlpha  ( initData(&useAlpha  , false, "useAlpha"  , "True if velocity displayed using alpha blending") )
, vscale    ( initData(&vscale    ,   1.0, "vscale"    , "Scaling of veloity vectors") )
{
}

void DrawV::initTextures()
{
}

void DrawV::update()
{
}

bool DrawV::isTransparent()
{
    return useAlpha.getValue();
}

void DrawV::draw()
{
    if (!isTransparent())
	internalDraw();
}

void DrawV::drawTransparent()
{
    if (isTransparent())
	internalDraw();
}

void DrawV::drawShadow()
{
    if (!isTransparent() && castShadow.getValue())
    {
	//std::cout << "drawShadow for "<<getName()<<std::endl;
	internalDraw();
    }
}

float DrawV::getVScale()
{
    return (float)vscale.getValue();
    //return 1.0f;
    //return getContext()->getDt();
}

void DrawV::internalDraw()
{
    if (!getContext()->getShowVisualModels()) return;

    glDisable(GL_LIGHTING);

    if (isTransparent()) {
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(GL_FALSE);
    }

    const unsigned int nbp = inputX.size();
    const float vscale = getVScale();
    glBegin(GL_LINES);
    for (unsigned int i=0;i<nbp;i++)
    {
	const Coord& p = inputX[i];
	const Coord p1 = inputX[i] + inputV[i]*vscale;
	const Coord p2 = inputX[i] - inputV[i]*vscale;
	glColor4f(1.0, 0.5, 0.5, 1.0);
	glVertex3fv(p.ptr());
	glColor4f(1.0, 0.5, 0.5, 0.0);
	glVertex3fv(p1.ptr());
	glColor4f(0.5, 1.0, 0.5, 1.0);
	glVertex3fv(p.ptr());
	glColor4f(0.5, 1.0, 0.5, 0.0);
	glVertex3fv(p2.ptr());
    }
    glEnd();

    glDisable(GL_LIGHTING);

    if (isTransparent()) {
	glDisable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glBlendFunc(GL_ONE, GL_ZERO);
	glDepthMask(GL_TRUE);
    }
}

bool DrawV::addBBox(double* minBBox, double* maxBBox)
{
    const unsigned int nbp = inputX.size();
    const float vscale = getVScale();
    for (unsigned int i=0; i<nbp; i++)
    {
	const Coord& p = inputX[i];
	const Coord p1 = inputX[i]+inputV[i]*vscale;
	const Coord p2 = inputX[i]-inputV[i]*vscale;
	for (int c=0;c<3;c++)
	{
	    if (p[c] > maxBBox[c]) maxBBox[c] = p[c];
	    if (p[c] < minBBox[c]) minBBox[c] = p[c];
	    if (p1[c] > maxBBox[c]) maxBBox[c] = p1[c];
	    if (p1[c] < minBBox[c]) minBBox[c] = p1[c];
	    if (p2[c] > maxBBox[c]) maxBBox[c] = p2[c];
	    if (p2[c] < minBBox[c]) minBBox[c] = p2[c];
	}
  }  
  return true;
}

} // namespace visualmodel

} // namespace component

} // namespace sofa

