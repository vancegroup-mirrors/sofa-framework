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
#ifndef SOFA_COMPONENT_COLLISION_SPHEREMODEL_INL
#define SOFA_COMPONENT_COLLISION_SPHEREMODEL_INL

#include <sofa/component/collision/SphereModel.h>
#include <sofa/helper/io/SphereLoader.h>
#include <sofa/component/collision/CubeModel.h>
#include <sofa/helper/system/FileRepository.h>
#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>

namespace sofa
{

namespace component
{

namespace collision
{

template<class TDataTypes>
TSphereModel<TDataTypes>::TSphereModel()
: radius(initData(&radius, "listRadius","Radius of each sphere")), 
  defaultRadius(initData(&defaultRadius,(SReal)(1.0), "radius","Default Radius"))
{
  if (radius.getValue().size() == 0)
  {
    resize(getSize()); // make sure the CollisionModel and the MechanicalObject have the same size
    for(unsigned int i=0;i<this->radius.getValue().size();i++) setRadius(i, -1);
  }
}


template<class TDataTypes>
TSphereModel<TDataTypes>::TSphereModel(SReal r)
  : radius(initData(&radius, "listRadius","Radius of each sphere")), 
    defaultRadius(initData(&defaultRadius, r,"radius","Default Radius"))
{
    resize(getSize()); // make sure the CollisionModel and the MechanicalObject have the same size
    for(unsigned int i=0;i<this->radius.getValue().size();i++) setRadius(i, r);
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::init()
{
    Inherit::init();
    for(unsigned int i=0;i<this->radius.getValue().size();i++)
      if (getRadius(i)==-1) setRadius(i,defaultRadius.getValue());
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::resize(int size)
{
	this->component::MechanicalObject<InDataTypes>::resize(size);
	this->core::CollisionModel::resize(size);
	if ((int)radius.getValue().size() < size)
	{
		radius.beginEdit()->reserve(size);
		while ((int)radius.getValue().size() < size)
			radius.beginEdit()->push_back(defaultRadius.getValue());
	}
	else
	{
		radius.beginEdit()->resize(size);
	}
}

template<class TDataTypes>
int TSphereModel<TDataTypes>::addSphere(const Vector3& pos, SReal radius)
{
	int i = size;
	resize(i+1);
	setSphere(i, pos, radius);
	return i;
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::setSphere(int i, const Vector3& pos, SReal r)
{
	if ((unsigned)i >= (unsigned) size) return;
	(*this->getX())[i] = pos;
	setRadius(i,r);
}

template<class TDataTypes>
class TSphereModel<TDataTypes>::Loader : public helper::io::SphereLoader
{
public:
	TSphereModel<TDataTypes>* dest;
	Loader(TSphereModel<TDataTypes>* dest) : dest(dest) { }
	void addSphere(SReal x, SReal y, SReal z, SReal r)
	{
		dest->addSphere(Vector3(x,y,z),r);
	}
};

template<class TDataTypes>
bool TSphereModel<TDataTypes>::load(const char* filename)
{
      	this->resize(0);
        std::string sphereFilename(filename);       
        if (!sofa::helper::system::DataRepository.findFile (sphereFilename))                 
          logWarning(std::string("Sphere File \"") + filename +std::string("\" not found")); 
	 Loader loader(this);
	 return loader.load(filename);
        
        
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::applyScale(SReal s)
{
	Inherit::applyScale(s);
	//std::cout << "Applying scale " << s << " to " << size << " spheres" << std::endl;
	for (int i=0;i<size;i++) setRadius(i, getRadius(i)*s);
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::draw(int index)
{
	const typename TDataTypes::VecCoord& x = *(((const TSphereModel<TDataTypes>*)this)->getX());
	Vector3 p = x[index];
	glPushMatrix();
	glTranslated(p[0], p[1], p[2]);
	//glutSolidSphere(radius[index], 8, 4);
	glutSolidSphere(getRadius(index), 32,16);
	glPopMatrix();
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::draw()
{
	if (getContext()->getShowCollisionModels())
	{
		glEnable(GL_LIGHTING);
		glEnable(GL_COLOR_MATERIAL);
		//glDisable(GL_LIGHTING);
		glColor4fv(getColor4f());
		for (int i=0;i<size;i++)
		{
			draw(i);
		}
		glDisable(GL_LIGHTING);
		glDisable(GL_COLOR_MATERIAL);
	}
	if (getPrevious()!=NULL && getContext()->getShowBoundingCollisionModels())
		getPrevious()->draw();
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::computeBoundingTree(int maxDepth)
{
	CubeModel* cubeModel = createPrevious<CubeModel>();
	if (!isMoving() && !cubeModel->empty()) return; // No need to recompute BBox if immobile
	
	Vector3 minElem, maxElem;

	cubeModel->resize(size);
	if (!empty())
	{
		const typename TDataTypes::VecCoord& x = *(((const TSphereModel<TDataTypes>*)this)->getX());
		for (int i=0;i<size;i++)
		{
			SReal r = getRadius(i);
			for (int c=0;c<3;c++)
			{
				minElem[c] = x[i][c] - r;
				maxElem[c] = x[i][c] + r;
			}
			cubeModel->setParentOf(i, minElem, maxElem);
		}
		cubeModel->computeBoundingTree(maxDepth);
	}
}

template<class TDataTypes>
void TSphereModel<TDataTypes>::computeContinuousBoundingTree(double dt, int maxDepth)
{
	CubeModel* cubeModel = createPrevious<CubeModel>();
	if (!isMoving() && !cubeModel->empty()) return; // No need to recompute BBox if immobile
	
	Vector3 minElem, maxElem;

	cubeModel->resize(this->size);
	if (!empty())
	{
		const typename TDataTypes::VecCoord& x = *this->getX();
		const typename TDataTypes::VecDeriv& v = *this->getV();
		for (int i=0;i<this->size;i++)
		{
			SReal r = getRadius(i);
			for (int c=0;c<3;c++)
			{
				if (v[i][c] < 0)
				{
					minElem[c] = x[i][c] + v[i][c]*dt - r;
					maxElem[c] = x[i][c]           + r;
				}
				else
				{
					minElem[c] = x[i][c]           - r;
					maxElem[c] = x[i][c] + v[i][c]*dt + r;
				}
			}
			cubeModel->setParentOf(i, minElem, maxElem);
		}
		cubeModel->computeBoundingTree(maxDepth);
	}
}

} // namespace collision

} // namespace component

} // namespace sofa

#endif
