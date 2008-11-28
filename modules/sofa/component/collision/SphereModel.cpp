/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 3      *
*                (c) 2006-2008 MGH, INRIA, USTL, UJF, CNRS                    *
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
#include <sofa/component/collision/proximity.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/componentmodel/collision/Intersection.inl>
#include <iostream>
#include <algorithm>

#include <sofa/helper/io/SphereLoader.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/component/collision/SphereModel.h>
#include <sofa/component/collision/CubeModel.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>

#include <sofa/core/componentmodel/topology/BaseMeshTopology.h>

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace sofa::core::componentmodel::collision;
using namespace helper;

SOFA_DECL_CLASS(Sphere)

int SphereModelClass = core::RegisterObject("Collision model which represents a set of Spheres")
        .add< SphereModel >()
        .addAlias("Sphere")
//.addAlias("SphereModel")
//.addAlias("SphereMesh")
//.addAlias("SphereSet")
        ;

SphereModel::SphereModel()
    : mstate(NULL)
    , radius(initData(&radius, "listRadius","Radius of each sphere"))
    , defaultRadius(initData(&defaultRadius,(SReal)(1.0), "radius","Default Radius"))
{
}

void SphereModel::resize(int size)
{
    this->core::CollisionModel::resize(size);

    if((int) radius.getValue().size() < size)
    {
        radius.beginEdit()->reserve(size);
        while((int)radius.getValue().size() < size)
            radius.beginEdit()->push_back(defaultRadius.getValue());
    }
    else
    {
        radius.beginEdit()->resize(size);
    }
}

void SphereModel::parse(BaseObjectDescription* arg)
{
    this->core::CollisionModel::parse(arg);

    if (arg->getAttribute("filename"))
    {
        filename.setValue(arg->getAttribute("filename"));
    }
}

void SphereModel::init()
{
    this->CollisionModel::init();
    mstate = dynamic_cast< core::componentmodel::behavior::MechanicalState<Vec3Types>* > (getContext()->getMechanicalState());

    if (mstate==NULL)
    {
        logWarning("SphereModel requires a Vec3 Mechanical Model");
        return;
    }

    if (!filename.getValue().empty())
    {
        load(filename.getValue().c_str());
    }
    else
    {
        const int npoints = mstate->getX()->size();
        resize(npoints);
    }
}

void SphereModel::draw(int index)
{
    Sphere t(this,index);

    Vector3 p = t.p();
    glPushMatrix();
    glTranslated(p[0], p[1], p[2]);
    glutSolidSphere(t.r(), 32, 16);
    glPopMatrix();
}

void SphereModel::draw()
{
    if (getContext()->getShowCollisionModels())
    {
        glEnable(GL_LIGHTING);
        glEnable(GL_COLOR_MATERIAL);
        glColor4fv(getColor4f());

        // Check topological modifications
        const int npoints = mstate->getX()->size();

        for (int i=0; i<npoints; i++)
        {
            draw(i);
        }

        glDisable(GL_LIGHTING);
        glDisable(GL_COLOR_MATERIAL);
    }
    if (getPrevious()!=NULL && getContext()->getShowBoundingCollisionModels())
        getPrevious()->draw();
}

void SphereModel::computeBoundingTree(int maxDepth)
{
    CubeModel* cubeModel = createPrevious<CubeModel>();
    const int npoints = mstate->getX()->size();
    bool updated = false;
    if (npoints != size)
    {
        resize(npoints);
        updated = true;
        cubeModel->resize(0);
    }

    if (!isMoving() && !cubeModel->empty() && !updated)
        return; // No need to recompute BBox if immobile

    cubeModel->resize(size);
    if (!empty())
    {
        for (int i=0; i<size; i++)
        {
            Sphere p(this,i);
            const Sphere::Real r = p.r();
            const Vector3 minElem = p.center() - Vector3(r,r,r);
            const Vector3 maxElem = p.center() + Vector3(r,r,r);

            cubeModel->setParentOf(i, minElem, maxElem);

        }
        cubeModel->computeBoundingTree(maxDepth);
    }
}

void SphereModel::computeContinuousBoundingTree(double dt, int maxDepth)
{
    CubeModel* cubeModel = createPrevious<CubeModel>();
    const int npoints = mstate->getX()->size();
    bool updated = false;
    if (npoints != size)
    {
        resize(npoints);
        updated = true;
        cubeModel->resize(0);
    }

    if (!isMoving() && !cubeModel->empty() && !updated)
        return; // No need to recompute BBox if immobile

    Vector3 minElem, maxElem;

    cubeModel->resize(size);
    if (!empty())
    {
        for (int i=0; i<size; i++)
        {
            Sphere p(this,i);
            const Vector3& pt = p.p();
            const Vector3 ptv = pt + p.v()*dt;

            for (int c = 0; c < 3; c++)
            {
                minElem[c] = pt[c];
                maxElem[c] = pt[c];
                if (ptv[c] > maxElem[c]) maxElem[c] = ptv[c];
                else if (ptv[c] < minElem[c]) minElem[c] = ptv[c];
            }

            Sphere::Real r = p.r();
            cubeModel->setParentOf(i, minElem - Vector3(r,r,r), maxElem + Vector3(r,r,r));
        }
        cubeModel->computeBoundingTree(maxDepth);
    }
}

SphereModel::Real SphereModel::getRadius(const int i) const
{
    if(i < (int) this->radius.getValue().size())
        return radius.getValue()[i];
    else
        return defaultRadius.getValue();
}

void SphereModel::setRadius(const int i, const SphereModel::Real r)
{
    if((int) radius.getValue().size() <= i)
    {
        radius.beginEdit()->reserve(i+1);
        while((int)radius.getValue().size() <= i)
            radius.beginEdit()->push_back(defaultRadius.getValue());
    }

    (*radius.beginEdit())[i] = r;
}

void SphereModel::setRadius(const SphereModel::Real r)
{
    *defaultRadius.beginEdit() = r;
    radius.beginEdit()->clear();
}

class SphereModel::Loader : public helper::io::SphereLoader
{
public:
    SphereModel* dest;
    Loader(SphereModel* dest)
        : dest(dest) {}

    void addSphere(SReal x, SReal y, SReal z, SReal r)
    {
        dest->addSphere(Vector3(x,y,z),r);
    }
};

bool SphereModel::load(const char* filename)
{
    this->resize(0);
    std::string sphereFilename(filename);
    if (!sofa::helper::system::DataRepository.findFile (sphereFilename))
        logWarning(std::string("Sphere File \"") + filename +std::string("\" not found"));

    Loader loader(this);
    return loader.load(filename);
}

int SphereModel::addSphere(const Vector3& pos, Real r)
{
    int i = size;
    resize(i+1);
    if((int) mstate->getX()->size() != i+1)
        mstate->resize(i+1);

    setSphere(i, pos, r);
    return i;
}

void SphereModel::setSphere(int i, const Vector3& pos, Real r)
{
    (*mstate->getX())[i] = pos;
    setRadius(i,r);
}

} // namespace collision

} // namespace component

} // namespace sofa

