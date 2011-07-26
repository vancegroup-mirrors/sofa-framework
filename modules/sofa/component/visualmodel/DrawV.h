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
#ifndef SOFA_COMPONENT_VISUALMODEL_DRAWV_H
#define SOFA_COMPONENT_VISUALMODEL_DRAWV_H

#include <string>
#include <sofa/helper/gl/template.h>
#include <sofa/core/visual/VisualModel.h>
#include <sofa/core/State.h>
#include <sofa/component/component.h>
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

class SOFA_COMPONENT_VISUALMODEL_API DrawV : public core::visual::VisualModel, public core::State< ExtVec3fTypes >
{
public:
    SOFA_CLASS2(DrawV, core::visual::VisualModel, SOFA_TEMPLATE(core::State, ExtVec3fTypes));

    Data<bool> castShadow; ///< True if object cast shadows
    Data<bool> useAlpha; ///< True if velocity displayed using alpha blending
    Data<double> vscale; ///< Scaling of veloity vectors

    DrawV();

    void initVisual();
    void updateVisual();

    bool isTransparent();
    float getVScale();

    void drawVisual();
    void drawTransparent();
    void drawShadow();

    bool addBBox(double* minBBox, double* maxBBox);

    virtual void resize(int vsize) { inputX.resize( vsize); inputV.resize( vsize);}
	virtual int getSize() const { return 0; }

    const VecCoord* getX()  const { return &inputX; }
    const VecDeriv* getV()  const { return &inputV; }
    VecCoord* getX()  { return &inputX; }
    VecCoord* getV()  { return &inputV; }

    VecCoord* getX0() { return NULL; }
    VecCoord* getN() { return NULL; }

    const VecCoord* getX0() const { return NULL; }
    const VecDeriv* getN() const { return NULL; }

    virtual       Data<VecCoord>* write(     core::VecCoordId /* v */) { return NULL; }
    virtual const Data<VecCoord>*  read(core::ConstVecCoordId /* v */) const { return NULL; }

    virtual       Data<VecDeriv>* write(     core::VecDerivId /* v */) { return NULL; }
    virtual const Data<VecDeriv>*  read(core::ConstVecDerivId /* v */) const { return NULL; }

    virtual       Data<MatrixDeriv>* write(     core::MatrixDerivId /* v */) { return NULL; }
    virtual const Data<MatrixDeriv>*  read(core::ConstMatrixDerivId /* v */) const {  return NULL; }

protected:
    ResizableExtVector<Coord> inputX;
    ResizableExtVector<Deriv> inputV;

    void internalDraw();
};


} // namespace visualmodel

} // namespace component

} // namespace sofa

#endif
