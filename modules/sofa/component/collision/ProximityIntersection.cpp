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
#include <sofa/helper/system/config.h>
#include <sofa/component/collision/ProximityIntersection.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/component/collision/proximity.h>
#include <sofa/core/componentmodel/collision/Intersection.inl>
#include <sofa/component/collision/RayPickInteractor.h>
#include <iostream>
#include <algorithm>

namespace sofa
{

namespace component
{

namespace collision
{

using namespace sofa::defaulttype;
using namespace sofa::core::componentmodel::collision;
using namespace helper;


SOFA_DECL_CLASS(ProximityIntersection)

int ProximityIntersectionClass = core::RegisterObject("TODO")
.add< ProximityIntersection >()
;

ProximityIntersection::ProximityIntersection()
: useTriangleTriangle(dataField(&useTriangleTriangle, false, "useTriangleTriangle","TODO"))
, useLineTriangle(dataField(&useLineTriangle, true, "useLineTriangle","TODO"))
, usePointTriangle(dataField(&usePointTriangle, false, "usePointTriangle","TODO"))
, useSphereTriangle(dataField(&useSphereTriangle, true, "useSphereTriangle","TODO"))
, alarmDistance(dataField(&alarmDistance, 1.0, "alarmDistance","TODO"))
, contactDistance(dataField(&contactDistance, 0.5, "contactDistance","TODO"))
{
}

void ProximityIntersection::init()
{
	intersectors.add<CubeModel,     CubeModel,     ProximityIntersection,                 false>(this);
	if (useTriangleTriangle.getValue())
	{
		intersectors.add<TriangleModel, TriangleModel, ProximityIntersection, false>(this);
	}
	else
	{
		intersectors.ignore<TriangleModel, TriangleModel, false>();
	}
	if (useLineTriangle.getValue())
	{
		intersectors.add<LineModel, TriangleModel, ProximityIntersection, true>(this);
	}
	else
	{
		intersectors.ignore<LineModel, TriangleModel, true>();
	}
	if (usePointTriangle.getValue())
	{
		intersectors.add<PointModel, TriangleModel, ProximityIntersection, true>(this);
	}
	else
	{
		intersectors.ignore<PointModel, TriangleModel, true>();
	}
	if (useSphereTriangle.getValue())
	{
		intersectors.add<SphereModel, TriangleModel, ProximityIntersection, true>(this);
	}
	else
	{
		intersectors.ignore<SphereModel, TriangleModel, true>();
	}
	intersectors.ignore<PointModel, PointModel, false>();
	intersectors.ignore<LineModel, LineModel, false>();
	intersectors.ignore<PointModel, LineModel, true>();
	intersectors.add<RayModel, TriangleModel, ProximityIntersection, true>(this);
	intersectors.add<RayPickInteractor, TriangleModel, ProximityIntersection, true>(this);
}

//static ProximityIntersection* proximityInstance = NULL;

/// Return the intersector class handling the given pair of collision models, or NULL if not supported.
ElementIntersector* ProximityIntersection::findIntersector(core::CollisionModel* object1, core::CollisionModel* object2)
{
	//proximityInstance = this;
	return this->DiscreteIntersection::findIntersector(object1, object2);
}

bool ProximityIntersection::testIntersection(Cube &cube1, Cube &cube2)
{
	const Vector3& minVect1 = cube1.minVect();
	const Vector3& minVect2 = cube2.minVect();
	const Vector3& maxVect1 = cube1.maxVect();
	const Vector3& maxVect2 = cube2.maxVect();
	const double alarmDist = getAlarmDistance() + cube1.getProximity() + cube2.getProximity();

	for (int i=0;i<3;i++)
	{
		if ( minVect1[i] > maxVect2[i] + alarmDist || minVect2[i]> maxVect1[i] + alarmDist )
			return false;
	}

	return true;
}

int ProximityIntersection::computeIntersection(Cube&, Cube&, OutputVector*)
{
	return 0; /// \todo
}

bool ProximityIntersection::testIntersection(Triangle& t1, Triangle& t2)
{
	Vector3 P,Q,PQ;
	static DistanceTriTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	proximitySolver.NewComputation( &t1, &t2,P,Q);
	PQ = Q-P;

	if (PQ.norm2() < alarmDist*alarmDist)
	{
		//std::cout<<"Collision between Triangle - Triangle"<<std::endl;
		return true;
	}
	else
		return false;
}

int ProximityIntersection::computeIntersection(Triangle& t1, Triangle& t2, OutputVector* contacts)
{
	Vector3 P,Q,PQ;
	static DistanceTriTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	proximitySolver.NewComputation( &t1, &t2,P,Q);
	PQ = Q-P;

	if (PQ.norm2() >= alarmDist*alarmDist)
		return 0;

	const double contactDist = getContactDistance() + t1.getProximity() + t2.getProximity();
	contacts->resize(contacts->size()+1);
	DetectionOutput *detection = &*(contacts->end()-1);

	detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(t1, t2);
    detection->id = (t1.getCollisionModel()->getSize() > t2.getCollisionModel()->getSize()) ? t1.getIndex() : t2.getIndex();
	detection->point[0]=P;
	detection->point[1]=Q;
	detection->normal=PQ;
	detection->value = detection->normal.norm();
	detection->normal /= detection->value;
	if (detection->normal * t2.n() < -0.5)
	{ // The elements are interpenetrating
		detection->normal = -detection->normal;
		detection->value = -detection->value;
	}
	detection->value -= contactDist;
	return 1;

}

bool ProximityIntersection::testIntersection(Line& t1, Triangle& t2)
{
	Vector3 P,Q,PQ;
	static DistanceSegTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	if (fabs(t2.n() * (t1.p2()-t1.p1())) < 0.000001)
		return false; // no intersection for edges parallel to the triangle

	proximitySolver.NewComputation( &t2, t1.p1(), t1.p2(),P,Q);
	PQ = Q-P;

	if (PQ.norm2() < alarmDist*alarmDist)
	{
		//std::cout<<"Collision between Line - Triangle"<<std::endl;
		return true;
	}
	else
		return false;
}

int ProximityIntersection::computeIntersection(Line& t1, Triangle& t2, OutputVector* contacts)
{
	Vector3 P,Q,PQ;
	static DistanceSegTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	if (fabs(t2.n() * (t1.p2()-t1.p1())) < 0.000001)
		return 0; // no intersection for edges parallel to the triangle

	proximitySolver.NewComputation( &t2, t1.p1(), t1.p2(),P,Q);
	PQ = Q-P;

	if (PQ.norm2() >= alarmDist*alarmDist)
		return 0;

	const double contactDist = getContactDistance() + t1.getProximity() + t2.getProximity();
	contacts->resize(contacts->size()+1);
	DetectionOutput *detection = &*(contacts->end()-1);

	detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(t2, t1);
    detection->id = (t1.getCollisionModel()->getSize() > t2.getCollisionModel()->getSize()) ? t1.getIndex() : t2.getIndex();
	detection->point[0]=P;
	detection->point[1]=Q;
	detection->normal=PQ;
	detection->value = detection->normal.norm();
	detection->normal /= detection->value;
	if (t2.getCollisionModel()->isStatic() && detection->normal * t2.n() < -0.95)
	{ // The elements are interpenetrating
		detection->normal = -detection->normal;
		detection->value = -detection->value;
	}
	detection->value -= contactDist;
	return 1;
}

bool ProximityIntersection::testIntersection(Point& t1, Triangle& t2)
{
	Vector3 P,Q,PQ;
	static DistancePointTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	Q = t1.p();
	proximitySolver.NewComputation( &t2, Q,P);
	PQ = Q-P;

	if (PQ.norm2() < alarmDist*alarmDist)
	{
		//std::cout<<"Collision between Point - Triangle"<<std::endl;
		return true;
	}
	else
		return false;
}

int ProximityIntersection::computeIntersection(Point& t1, Triangle& t2, OutputVector* contacts)
{
	Vector3 P,Q,PQ;
	static DistancePointTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	Q = t1.p();
	proximitySolver.NewComputation( &t2, Q,P);
	PQ = Q-P;

	if (PQ.norm2() >= alarmDist*alarmDist)
		return 0;

	const double contactDist = getContactDistance() + t1.getProximity() + t2.getProximity();
	contacts->resize(contacts->size()+1);
	DetectionOutput *detection = &*(contacts->end()-1);

	detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(t2, t1);
    detection->id = t1.getIndex();
	detection->point[0]=P;
	detection->point[1]=Q;
	detection->normal=PQ;
	detection->value = detection->normal.norm();
	detection->normal /= detection->value;
	//if (detection->normal * t2.n() < -0.95)
	//{ // The elements are interpenetrating
	//	detection->normal = -detection->normal;
	//	detection->value = -detection->value;
	//}
	detection->value -= contactDist;
	return 1;

}

bool ProximityIntersection::testIntersection(Sphere& t1, Triangle& t2)
{
	Vector3 P,Q,PQ;
	static DistancePointTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.r() + t1.getProximity() + t2.getProximity();

	Q = t1.center();
	proximitySolver.NewComputation( &t2, Q,P);
	PQ = Q-P;

	if (PQ.norm2() < alarmDist*alarmDist)
	{
		//std::cout<<"Collision between Point - Triangle"<<std::endl;
		return true;
	}
	else
		return false;
}

int ProximityIntersection::computeIntersection(Sphere& t1, Triangle& t2, OutputVector* contacts)
{
	Vector3 P,Q,PQ;
	static DistancePointTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.r() + t1.getProximity() + t2.getProximity();

	Q = t1.center();
	proximitySolver.NewComputation( &t2, Q,P);
	PQ = Q-P;

	if (PQ.norm2() < alarmDist*alarmDist)
		return 0;

	const double contactDist = getContactDistance() + t1.r() + t1.getProximity() + t2.getProximity();
	contacts->resize(contacts->size()+1);
	DetectionOutput *detection = &*(contacts->end()-1);

	detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(t2, t1);
    detection->id = t1.getIndex();
	detection->point[0]=P;
	detection->point[1]=Q;
	detection->normal=PQ;
	detection->value = detection->normal.norm();
	detection->normal /= detection->value;
	//if (detection->normal * t2.n() < -0.95)
	//{ // The elements are interpenetrating
	//	detection->normal = -detection->normal;
	//	detection->value = -detection->value;
	//}
	detection->value -= contactDist;
	return 1;

}

bool ProximityIntersection::testIntersection(Ray &t1,Triangle &t2)
{

	if (fabs(t2.n() * t1.direction()) < 0.000001)
		return false; // no intersection for edges parallel to the triangle

	Vector3 A = t1.origin();
	Vector3 B = A + t1.direction() * t1.l();

	Vector3 P,Q,PQ;
	static DistanceSegTri proximitySolver;
	const double alarmDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	proximitySolver.NewComputation( &t2, A, B,P,Q);
	PQ = Q-P;

	if (PQ.norm2() < alarmDist*alarmDist)
	{
		//std::cout<<"Collision between Line - Triangle"<<std::endl;
		return true;
	}
	else
		return false;
}

int ProximityIntersection::computeIntersection(Ray &t1, Triangle &t2, OutputVector* contacts)
{

	if (fabs(t2.n() * t1.direction()) < 0.000001)
		return 0; // no intersection for edges parallel to the triangle

	Vector3 A = t1.origin();
	Vector3 B = A + t1.direction() * t1.l();

	Vector3 P,Q,PQ;
	static DistanceSegTri proximitySolver;
	const double alarmDist = 0.01; //getAlarmDistance();

	proximitySolver.NewComputation( &t2, A, B,P,Q);
	PQ = Q-P;

	if (PQ.norm2() >= alarmDist*alarmDist)
		return 0;
	
	const double contactDist = getAlarmDistance() + t1.getProximity() + t2.getProximity();

	proximitySolver.NewComputation( &t2, A,B,P,Q);
	contacts->resize(contacts->size()+1);
	DetectionOutput *detection = &*(contacts->end()-1);

	detection->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(t2, t1);
    detection->id = t1.getIndex();
	detection->point[0]=P;
	detection->point[1]=Q;
	detection->normal=t2.n();
	detection->value = (PQ).norm();
	detection->value -= contactDist;
	return 1;
}

} // namespace collision

} // namespace component

} // namespace sofa

