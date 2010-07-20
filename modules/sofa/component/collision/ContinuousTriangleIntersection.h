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
#ifndef SOFA_COMPONENT_COLLISION_CONTINUOUSTRIANGLEINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_CONTINUOUSTRIANGLEINTERSECTION_H
#include <sofa/component/collision/Triangle.h>
#include <sofa/core/collision/DetectionOutput.h>
#include <math.h>



namespace sofa
{

namespace component
{

namespace collision
{

//#define cbrt(x)     ((x) > (SReal)0.0 ? pow((SReal)(x), (SReal)1.0/(SReal)3.0) : 
//	((x) < (SReal)0.0 ? -pow((SReal)-(x), (SReal)1.0/(SReal)3.0) : (SReal)0.0))

//#define	IsZero(x)	((x) > -EQN_EPS && (x) < EQN_EPS)

using namespace collision;

class SOFA_COMPONENT_COLLISION_API ContinuousTriangleIntersection
{
private:
	Triangle &tr1, &tr2;
	SReal m_tolerance, m_tolmin, m_tolmax;


	int intersectPointTriangle (SReal& t, SReal& u, SReal& v,
                                const Vector3& p0, const Vector3& v0,
                                const Vector3& p1, const Vector3& v1,
                                const Vector3& p2, const Vector3& v2,
                                const Vector3& p3, const Vector3& v3,
                                double dt);

    int intersectEdgeEdge(SReal& t, SReal& u, SReal& v,
                           const Vector3& p1, const Vector3& p2,
                           const Vector3& v1, const Vector3& v2,
                           const Vector3& p3, const Vector3& p4,
                           const Vector3& v3, const Vector3& v4,
                           double dt);

	int solveCubic(Vector3& s, SReal c[4]);
    
    int solveQuadratic (SReal& t1, SReal& t2, 
			const SReal& a, 
			const SReal& b, 
			const SReal& c);
    
    bool checkValidRoots (SReal& validRoot,
                          const int& numRoots,
                          const Vector3& roots,
                          const SReal& rootMin,
                          const SReal& rootMax);

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns true of value lies within the range
    bool checkRange (const SReal& val,
                     const SReal& min,
                     const SReal& max)
    {
        return ((val >= min) && (val <= max));
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns the smallest of the two values
    inline SReal getMinVal (const SReal& val0, const SReal& val1)
    {
		return (val0 < val1) ? val0 : val1;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns the smallest of the three values
   SReal getMinVal(const SReal &val0,
							const SReal &val1,
							const SReal &val2)
    {
        if (val0 < val1)
        {
			return (val0 < val2) ? val0 : val2;
        }
        else 
		{
			return (val1 < val2) ? val1 : val2;
		}
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// sets tolerance for boundary check [-tol, 1+tol]
    void setTolerance(const SReal &tol)
    {
        m_tolerance = tol;
		m_tolmin = -m_tolerance;
		m_tolmax = (SReal)1.0 + m_tolerance;
    };

public:
	ContinuousTriangleIntersection (Triangle& t1, Triangle& t2);
	~ContinuousTriangleIntersection(void);
	core::collision::DetectionOutput* computeDetectionOutput (void);
	bool isCollision(void);

	static const SReal EPSILON;
	static const SReal EQN_EPS;

	static bool IsZero(SReal x){	return((x) > -EQN_EPS && (x) < EQN_EPS); }

	static SReal cbrt(SReal x) {  
		return  ((x) > (SReal)0.0 ? pow(x, (SReal)1.0/(SReal)3.0) : (x < (SReal)0.0 ? -pow(-x, (SReal)1.0/(SReal)3.0) : (SReal)0.0));
	}

};

} // namespace collision

} // namespace component

} // namespace sofa


#endif /* _CONTINUOUSTRIANGLEINTERSECTION_H_ */
