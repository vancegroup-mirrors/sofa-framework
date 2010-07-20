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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_BEAMCONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINTSET_BEAMCONSTRAINT_INL

#include <sofa/component/constraintset/BeamConstraint.h>
#include <sofa/component/constraintset/BilateralInteractionConstraint.h>

#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/gl/template.h>
namespace sofa
{

namespace component
{

namespace constraintset
{

template<class DataTypes>
void BeamConstraint<DataTypes>::init()
{
	assert(this->object1);
	assert(this->object2);
}

template<class DataTypes>
void BeamConstraint<DataTypes>::reset()
{
	internalInit();
}

template<class DataTypes>
void BeamConstraint<DataTypes>::bwdInit()
{
	internalInit();
}

template<class DataTypes>
void BeamConstraint<DataTypes>::internalInit()
{	// we search for the closest segment, on which to project each point
	
	int m1 = this->object1->getSize();
	int m2 = this->object2->getSize();
	
	previousSegment.clear();
	previousSegment.resize(m2);
	projected.clear();
	projected.resize(m2);
		
	Coord A, B, P, proj;

	for(int i=0; i<m2; i++)
	{
		P = (*this->object2->getX())[i];
		double closestDist = -1;
		if(i)
			previousSegment[i] = previousSegment[i-1];
		else
			previousSegment[0] = 0;
		projected[i] = false;
		
		for(int j=0; j<m1-1; j++)
		{
			A = (*this->object1->getX())[j];
			B = (*this->object1->getX())[j+1];
			
			Real r = ((P-A) * (B-A)) / (B-A).norm2();
			
			if(r>=0 && r<=1)
			{
				proj = A + (B-A) * r;
				double dist = (P-proj).norm2();
				if(closestDist<0 || dist<closestDist)
				{
					previousSegment[i] = j;
					projected[i] = true;
					closestDist = dist;
				}
			}
		}
	}
	
	sout << sendl;
}

template<class DataTypes>
void BeamConstraint<DataTypes>::getOrthogonalVectors(const Coord& dir, Coord& vec1, Coord& vec2)
{
	Coord temp;	// Any vector such as temp != dir
	temp[0] = dir[1];
	temp[1] = dir[2];
	temp[2] = dir[0];
	
	if(temp == dir) // x = y = z
		temp = Coord(1,0,0);
	
	vec1 = cross(dir, temp);
	vec1.normalize();
	
	vec2 = cross(dir, vec1);
	vec2.normalize();
}

template<class DataTypes>
typename DataTypes::Coord::value_type BeamConstraint<DataTypes>::checkProjection(const int point, const int segment)
{	
	Coord P = (*this->object2->getXfree())[point];
	
	Coord A = (*this->object1->getXfree())[segment];
	Coord B = (*this->object1->getXfree())[segment+1];
	
	return (Real)(((P-A) * (B-A)) / (B-A).norm2());
}

template<class DataTypes>
void BeamConstraint<DataTypes>::createConstraints(int point, int segment, int& nbConstraints, bool firstPoint)
{
	Coord A, B, P, uniAB, dir1, dir2, proj;
	
	P = (*this->object2->getXfree())[point];
	A = (*this->object1->getXfree())[segment];
	B = (*this->object1->getXfree())[segment+1];
	
	uniAB = (B - A);
	uniAB.normalize();
	
	Real r = (P-A) * uniAB;
	proj = A + uniAB * r;
	r /= (A-B).norm();
			
	dir1 = P-proj;
	Real dist = dir1.norm(); // constraint violation
	if(!dist)  // the point is on the line, we create 2 random vectors orthogonal to AB
	{	
		getOrthogonalVectors(uniAB, dir1, dir2);
	}
	else
	{	
		dir1.normalize(); // direction of the constraint	
		
		dir2 = cross(dir1, uniAB);
		dir2.normalize();
	}
	
	VecConst& c1 = *this->object1->getC();
	VecConst& c2 = *this->object2->getC();

	SparseVecDeriv svd1;
	SparseVecDeriv svd2;
	
	this->object1->setConstraintId(cid + nbConstraints);
	svd1.add(segment, -dir1 * (1-r));
	svd1.add(segment+1, -dir1 * r);
	c1.push_back(svd1);

	this->object2->setConstraintId(cid + nbConstraints);
	svd2.add(point, dir1);
	c2.push_back(svd2);
	dists.push_back(dist);
	nbConstraints++;
	
	this->object1->setConstraintId(cid + nbConstraints);
	svd1.set(segment, -dir2 * (1-r));
	svd1.set(segment+1, -dir2 * r);
	c1.push_back(svd1);

	this->object2->setConstraintId(cid + nbConstraints);
	svd2.set(point, dir2);
	c2.push_back(svd2);
	dists.push_back(0.0);
	nbConstraints++;
	
	if(firstPoint && autoAdvance.getValue())
	{
		this->object2->setConstraintId(cid + nbConstraints);
		svd2.set(point, -uniAB);
		c2.push_back(svd2);
		
		this->object1->setConstraintId(cid + nbConstraints);
		svd1.set(segment, uniAB * (1-r));
		svd1.set(segment+1, uniAB * r);
		c1.push_back(svd1);
		
		sofa::core::objectmodel::BaseContext* context = this->getContext(); 
		dists.push_back((Real)(autoAdvance.getValue()*context->getDt() - uniAB*(P-(*this->object2->getX())[point])));
		nbConstraints++;
	}

	projected[point] = true;
	previousSegment[point] = segment;
}

template<class DataTypes>
void BeamConstraint<DataTypes>::buildConstraintMatrix(unsigned int &constraintId, core::VecId)
{
	cid = constraintId;

	nbConstraints = 0;
	
	int m1, m2; // number of points in each object
	m1 = this->object1->getSize();
	m2 = this->object2->getSize();
		
	dists.clear();
	
	bool disconnected = true;
	for(int i=0; i<m2; i++)
		if(projected[i]) disconnected = false;
	if(disconnected)
		return;

	// we will try to project each point of the second object onto one segment of the first object
	for(int i=0; i<m2; i++)
	{
		projected[i] = false;
		
		// we test if the point was disconnected, and can't reconnect now
		if(previousSegment[i] < 0 	// disconnected before the first point
				 && (i+1 >= m2		// no neighbours in this direction
				 || previousSegment[i+1] < 0) ) // this neighbour is also disconnected
			continue;
		
		if(previousSegment[i] >= m1-1 	// disconnected after the last point
				 && (i-1 < 0			// no neighbours in this direction
				 || previousSegment[i-1] >= m1-1) ) // this neighbour is also disconnected
			continue;
		
		// We start looking for the projection of the point on the segment it was the previous timestep
		Real current;
		if(previousSegment[i] < 0)
			current = 2;
		if(previousSegment[i] >= m1-1)
			current = -1;
		else
			current = checkProjection(i, previousSegment[i]);
		
		if(current >= 0 && current <= 1)
			createConstraints(i, previousSegment[i], nbConstraints, i==0);
		else if(current < 0)
		{
			if(previousSegment[i]-1 < 0) // no segment there
				continue;
			Real prev = checkProjection(i, previousSegment[i]-1);
			if(prev > 1)
				createConstraints(i, previousSegment[i], nbConstraints, i==0); // we keep current segment
			else if(prev >= 0)
				createConstraints(i, previousSegment[i]-1, nbConstraints, i==0); // we take the previous segment
			else // prev < 0
			{
				// continue searching
				for(int j=2; previousSegment[i]-j >= 0; j++)
				{
					prev = checkProjection(i, previousSegment[i]-j);
					if(prev > 1)
					{
						createConstraints(i, previousSegment[i]-j+1, nbConstraints, i==0);
						break;
					}
					else if(prev >= 0)
					{
						createConstraints(i, previousSegment[i]-j, nbConstraints, i==0);
						break;
					}
				}
				
				// Didn't find any valid projection
			}
		}
		else // current > 1
		{
			if(previousSegment[i]+1 >= m1-1) // no segment there
				continue;
			Real next = checkProjection(i, previousSegment[i]+1);
			if(next < 0)
				createConstraints(i, previousSegment[i], nbConstraints, i==0); // we keep current segment
			else if(next <= 1)
				createConstraints(i, previousSegment[i]+1, nbConstraints, i==0); // we take the next segment
			else // next > 1
			{
				// continue searching
				for(int j=2; previousSegment[i]+j < m1-1; j++)
				{
					next = checkProjection(i, previousSegment[i]+j);
					if(next < 0)
					{
						createConstraints(i, previousSegment[i]+j-1, nbConstraints, i==0);
						break;
					}
					else if(next <= 1)
					{
						createConstraints(i, previousSegment[i]+j, nbConstraints, i==0);
						break;
					}
				}
			
				// Didn't find any valid projection
			}
		}
		
	}
	
	constraintId += nbConstraints;
}

template<class DataTypes>
void BeamConstraint<DataTypes>::getConstraintValue(defaulttype::BaseVector* v, bool freeMotion)
{
	if (!freeMotion)
		sout<<"WARNING has to be implemented for method based on non freeMotion"<<sendl;

	for(int i=0; i<nbConstraints; i++)
		v->set(cid+i, dists[i]);
}

template<class DataTypes>
void BeamConstraint<DataTypes>::getConstraintId(long* id, unsigned int &offset)
{
	if (!yetIntegrated)
	{
		id[offset++] = -(int)cid;

		yetIntegrated =  true;
	}
	else
	{
		id[offset++] = cid; 
	}
}

#ifdef SOFA_DEV
template<class DataTypes>
void BeamConstraint<DataTypes>::getConstraintResolution(std::vector<core::behavior::ConstraintResolution*>& resTab, unsigned int& offset)
{
	for(int i=0; i<nbConstraints; i++)
		resTab[offset++] = new BilateralConstraintResolution();
}
#endif

template<class DataTypes>
void BeamConstraint<DataTypes>::draw()
{
	sofa::core::objectmodel::BaseContext* context = this->getContext(); 
	if (!context->getShowInteractionForceFields()) return;

	glDisable(GL_LIGHTING);
	glPointSize(10);
	glBegin(GL_POINTS);
	int m = this->object1->getSize();
	glColor4f(0,0,1,1);
	for(int i=0; i<m; i++)
		helper::gl::glVertexT((*this->object1->getX())[i]);
	
	m = this->object2->getSize();
	for(int i=0; i<m; i++)
	{
		glColor4f(0.0f,1.0f,projected[i]?1:0.0f,1.0f);
		helper::gl::glVertexT((*this->object2->getX())[i]);
	}
	
	glEnd();
	
	glBegin(GL_LINES);
	
	m = this->object1->getSize();
	for(int i=1; i<m; i++)
	{
		float c = 1.0f/((float)(m-1)*(i-1));
		glColor4f(c,0.0f,1.0f-c,1.0f);
		helper::gl::glVertexT((*this->object1->getX())[i-1]);
		c = 1.0f/((float)(m-1)*i);
		glColor4f(c,0.0f,1.0f-c,1.0f);
		helper::gl::glVertexT((*this->object1->getX())[i]);
	}
	glEnd();
	glPointSize(1);
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
