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
#ifndef SOFA_COMPONENT_MASTERSOLVER_FREEMOTIONANIMATIONLOOP_H
#define SOFA_COMPONENT_MASTERSOLVER_FREEMOTIONANIMATIONLOOP_H

#include <sofa/simulation/common/CollisionAnimationLoop.h>

#include <sofa/component/component.h>

namespace sofa
{
namespace component
{
namespace constraintset
{
class LCPConstraintSolver;
} // namespace constraintset
} // namespace component
} // namespace sofa




namespace sofa
{

namespace component
{

namespace animationloop
{

class SOFA_CONSTRAINT_API FreeMotionAnimationLoop : public sofa::simulation::CollisionAnimationLoop
{
public:
    typedef sofa::simulation::CollisionAnimationLoop Inherit;

    SOFA_CLASS(FreeMotionAnimationLoop, sofa::simulation::CollisionAnimationLoop);

    FreeMotionAnimationLoop(simulation::Node* gnode);
    virtual ~FreeMotionAnimationLoop();

    virtual void step (const sofa::core::ExecParams* params /* PARAMS FIRST */, double dt);

    virtual void init();

    virtual void parse ( sofa::core::objectmodel::BaseObjectDescription* arg );

    /// Construction method called by ObjectFactory.
    template<class T>
    static void create(T*& obj, BaseContext* context, BaseObjectDescription* arg)
    {
        simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
        obj = new T(gnode);
        if (context) context->addObject(obj);
        if (arg) obj->parse(arg);
    }


    Data<bool> displayTime;

    Data<bool> m_solveVelocityConstraintFirst;

private :

    sofa::core::behavior::ConstraintSolver *constraintSolver;
    component::constraintset::LCPConstraintSolver* defaultSolver;
};

} // namespace animationloop

} // namespace component

} // namespace sofa

#endif /* SOFA_COMPONENT_MASTERSOLVER_FREEMOTIONANIMATIONLOOP_H */