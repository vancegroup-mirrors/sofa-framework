#ifndef SOFA_COMPONENT_VISUALMODEL_DRAWV_H
#define SOFA_COMPONENT_VISUALMODEL_DRAWV_H

#include <string>
#include <sofa/helper/gl/template.h>
#include <sofa/core/VisualModel.h>
#include <sofa/core/componentmodel/behavior/MappedModel.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Vec3Types.h>
#include "OglModel.h" // for ResizeableExtVector

namespace sofa
{

namespace component
{

namespace visualmodel
{

using namespace sofa::defaulttype;

class DrawV : public core::VisualModel, public core::componentmodel::behavior::MappedModel< ExtVectorTypes< Vec<3,GLfloat>, Vec<3,GLfloat> > >
{
public:
    Data<bool> castShadow; ///< True if object cast shadows
    Data<bool> useAlpha; ///< True if velocity displayed using alpha blending
    Data<double> vscale; ///< Scaling of veloity vectors
    
    DrawV();

    void initTextures();
    void update();

    bool isTransparent();
    float getVScale();
    
    void draw();
    void drawTransparent();
    void drawShadow();

    bool addBBox(double* minBBox, double* maxBBox);
	
    const VecCoord* getX()  const { return &inputX; }
    const VecDeriv* getV()  const { return &inputV; }
    VecCoord* getX()  { return &inputX; }
    VecDeriv* getV()  { return &inputV; }

protected:
    ResizableExtVector<Coord> inputX;
    ResizableExtVector<Deriv> inputV;

    void internalDraw();    
};


} // namespace visualmodel

} // namespace component

} // namespace sofa

#endif
