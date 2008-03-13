/*******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 1       *
*                (c) 2006-2007 MGH, INRIA, USTL, UJF, CNRS                     *
*                                                                              *
* This library is free software; you can redistribute it and/or modify it      *
* under the terms of the GNU Lesser General Public License as published by the *
* Free Software Foundation; either version 2.1 of the License, or (at your     *
* option) any later version.                                                   *
*                                                                              *
* This library is distributed in the hope that it will be useful, but WITHOUT  *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or        *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License  *
* for more details.                                                            *
*                                                                              *
* You should have received a copy of the GNU Lesser General Public License     *
* along with this library; if not, write to the Free Software Foundation,      *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.           *
*                                                                              *
* Contact information: contact@sofa-framework.org                              *
*                                                                              *
* Authors: J. Allard, P-J. Bensoussan, S. Cotin, C. Duriez, H. Delingette,     *
* F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza, M. Nesme, P. Neumann,        *
* and F. Poyer                                                                 *
*******************************************************************************/
#ifndef SOFA_CORE_VISUALMODEL_H
#define SOFA_CORE_VISUALMODEL_H

#include <sofa/core/objectmodel/BaseObject.h>

namespace sofa
{

namespace core
{

/**
 *  \brief An interface which all VisualModel inherit.
 *
 *  This Interface is used for the VisualModel, which all visible objects must
 *  implement.
 *
 *  VisualModels are drawn by calling their draw method. The method update is
 *  used to recompute some internal data (such as normals) after the simulation
 *  has computed a new timestep.
 *
 *  Most VisualModel are bound by a Mapping to a BehaviorModel or
 *  MechanicalState.
 */
class VisualModel : public virtual objectmodel::BaseObject
{
public:
    virtual ~VisualModel() { }

    /**
     *  \brief Initialize the textures, or other graphical resources.
     *
     *  Called once before the first frame is drawn, and if the graphical
     *  context has been recreated.
     */
    virtual void initTextures() = 0;

    /**
     *  \brief Display the VisualModel object.
     */
    virtual void draw() = 0;

    /**
     *  \brief Display transparent surfaces.
     *
     *  Objects should use this method to get a correct display order.
     */
    virtual void drawTransparent()
    {
    }

    /**
     *  \brief Display shadow-casting surfaces.
     *
     *  This method default to calling draw(). Object that do not cast any
     *  shadows, or that use a different LOD for them should reimplement it.
     */
    virtual void drawShadow()
    {
        draw();
    }

    /**
     *  \brief used to update the model if necessary.
     */
    virtual void update() = 0;

    /**
     *  \brief used to add the bounding-box of this visual model to the
     *  given bounding box in order to compute the scene bounding box or
     *  cull hidden objects.
     *
     *  \return false if the visual model does not define any bounding box,
     *  which should only be the case for "debug" objects, as this lack of
     *  information might affect performances and leads to incorrect scene
     *  bounding box.
     */
    virtual bool addBBox(double* /*minBBox*/, double* /*maxBBox*/)
    {
        return false;
    }

    /**
     *  \brief Append this mesh to an OBJ format stream.
     *
     *  The number of vertices position, normal, and texture coordinates already written is given as parameters.
     *  This method should update them.
     */
    virtual void exportOBJ(std::string /*name*/, std::ostream* /*out*/, std::ostream* /*mtl*/, int& /*vindex*/, int& /*nindex*/, int& /*tindex*/)
    {
    }
};

} // namespace core

} // namespace sofa

#endif
