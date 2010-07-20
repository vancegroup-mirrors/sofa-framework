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
#include <sofa/component/collision/ContinuousTriangleIntersection.h>
#include <sofa/helper/rmath.h>

namespace sofa
{

namespace component
{

namespace collision
{

	const SReal ContinuousTriangleIntersection::EPSILON = 0.000001;
	const SReal ContinuousTriangleIntersection::EQN_EPS = 1e-9;


bool ContinuousTriangleIntersection::isCollision(void)
{
	SReal t[3], u[3], v[3];
	double dt = 0.01; //Scene::getInstance()->getDt();
	//sout<<"Triangle 2  : " << tr2 << sendl;

//	getc(stdin);

	if (intersectPointTriangle(t[0], u[0], v[0], tr2.p1(), tr2.v1(), tr1.p1(), tr1.v1(), tr1.p2(), tr1.v2(), tr1.p3(), tr1.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectPointTriangle(t[1], u[1], v[1], tr2.p2(), tr2.v2(), tr1.p1(), tr1.v1(), tr1.p2(), tr1.v2(), tr1.p3(), tr1.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}
	
	if (intersectPointTriangle(t[2], u[2], v[2], tr2.p3(), tr2.v3(), tr1.p1(), tr1.v1(), tr1.p2(), tr1.v2(), tr1.p3(), tr1.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectPointTriangle(t[0], u[0], v[0], tr1.p1(), tr1.v1(), tr2.p1(), tr2.v1(), tr2.p2(), tr2.v2(), tr2.p3(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;	
	}

	if (intersectPointTriangle(t[1], u[1], v[1], tr1.p2(), tr1.v2(), tr2.p1(), tr2.v1(), tr2.p2(), tr2.v2(), tr2.p3(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;	
		return true;
	}

	if (intersectPointTriangle(t[2], u[2], v[2], tr1.p3(), tr1.v3(), tr2.p1(), tr2.v1(), tr2.p2(), tr2.v2(), tr2.p3(), tr2.v3(), dt))
	{
		 //sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[0], u[0], v[0], tr1.p1(), tr1.p2(), tr1.v1(), tr1.v2(), tr2.p1(), tr2.p2(), tr2.v1(), tr2.v2(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[1], u[1], v[1], tr1.p1(), tr1.p3(), tr1.v1(), tr1.v3(), tr2.p1(), tr2.p2(), tr2.v1(), tr2.v2(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}
	
	if (intersectEdgeEdge(t[2], u[2], v[2], tr1.p2(), tr1.p3(), tr1.v2(), tr1.v3(), tr2.p1(), tr2.p2(), tr2.v1(), tr2.v2(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[0], u[0], v[0], tr1.p1(), tr1.p2(), tr1.v1(), tr1.v2(), tr2.p1(), tr2.p3(), tr2.v1(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[1], u[1], v[1], tr1.p1(), tr1.p3(), tr1.v1(), tr1.v3(), tr2.p1(), tr2.p3(), tr2.v1(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[2], u[2], v[2], tr1.p2(), tr1.p3(), tr1.v2(), tr1.v3(), tr2.p1(), tr2.p3(), tr2.v1(), tr2.v3(), dt))
	{
		//sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[0], u[0], v[0], tr1.p1(), tr1.p2(), tr1.v1(), tr1.v2(), tr2.p2(), tr2.p3(), tr2.v2(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;
		return true;
	}

	if (intersectEdgeEdge(t[1], u[1], v[1], tr1.p1(), tr1.p3(), tr1.v1(), tr1.v3(), tr2.p2(), tr2.p3(), tr2.v2(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;	
		return true;
	}

	if (intersectEdgeEdge(t[2], u[2], v[2], tr1.p2(), tr1.p3(), tr1.v2(), tr1.v3(), tr2.p2(), tr2.p3(), tr2.v2(), tr2.v3(), dt))
	{
		// sout<<"Triangle 1  : " << tr1 << sendl;	
		return true;
	}

	return false;

}

core::collision::DetectionOutput* ContinuousTriangleIntersection::computeDetectionOutput (void)
{
	SReal t[3], u[3], v[3];
	double dt = 0.01; //Scene::getInstance()->getDt();

	for (int i = 0; i < 3; i++)
		t[i] = u[i] = v[i] = 0.0;

	intersectPointTriangle(t[0], u[0], v[0], tr2.p1(), tr2.v1(), tr1.p1(), tr1.v1(), tr1.p2(), tr1.v2(), tr1.p3(), tr1.v3(), dt);
	intersectPointTriangle(t[1], u[1], v[1], tr2.p2(), tr2.v2(), tr1.p1(), tr1.v1(), tr1.p2(), tr1.v2(), tr1.p3(), tr1.v3(), dt);
	intersectPointTriangle(t[2], u[2], v[2], tr2.p3(), tr2.v3(), tr1.p1(), tr1.v1(), tr1.p2(), tr1.v2(), tr1.p3(), tr1.v3(), dt);
	
	// sout << "t = "  << t[0] << " " << t[1] << " " << t[2] << sendl;

	intersectPointTriangle(t[0], u[0], v[0], tr1.p1(), tr1.v1(), tr2.p1(), tr2.v1(), tr2.p2(), tr2.v2(), tr2.p3(), tr2.v3(), dt);
	intersectPointTriangle(t[1], u[1], v[1], tr1.p2(), tr1.v2(), tr2.p1(), tr2.v1(), tr2.p2(), tr2.v2(), tr2.p3(), tr2.v3(), dt);
	intersectPointTriangle(t[2], u[2], v[2], tr1.p3(), tr1.v3(), tr2.p1(), tr2.v1(), tr2.p2(), tr2.v2(), tr2.p3(), tr2.v3(), dt);
	
	// sout << "t = "  << t[0] << " " << t[1] << " " << t[2] << sendl;

	intersectEdgeEdge(t[0], u[0], v[0], tr1.p1(), tr1.p2(), tr1.v1(), tr1.v2(), tr2.p1(), tr2.p2(), tr2.v1(), tr2.v2(), dt);
	intersectEdgeEdge(t[1], u[1], v[1], tr1.p1(), tr1.p3(), tr1.v1(), tr1.v3(), tr2.p1(), tr2.p2(), tr2.v1(), tr2.v2(), dt);
	intersectEdgeEdge(t[2], u[2], v[2], tr1.p2(), tr1.p3(), tr1.v2(), tr1.v3(), tr2.p1(), tr2.p2(), tr2.v1(), tr2.v2(), dt);
	
	// sout << "t = "  << t[0] << " " << t[1] << " " << t[2] << sendl;

	intersectEdgeEdge(t[0], u[0], v[0], tr1.p1(), tr1.p2(), tr1.v1(), tr1.v2(), tr2.p1(), tr2.p3(), tr2.v1(), tr2.v3(), dt);
	intersectEdgeEdge(t[1], u[1], v[1], tr1.p1(), tr1.p3(), tr1.v1(), tr1.v3(), tr2.p1(), tr2.p3(), tr2.v1(), tr2.v3(), dt);
	intersectEdgeEdge(t[2], u[2], v[2], tr1.p2(), tr1.p3(), tr1.v2(), tr1.v3(), tr2.p1(), tr2.p3(), tr2.v1(), tr2.v3(), dt);
	
	// sout << "t = "  << t[0] << " " << t[1] << " " << t[2] << sendl;

	intersectEdgeEdge(t[0], u[0], v[0], tr1.p1(), tr1.p2(), tr1.v1(), tr1.v2(), tr2.p2(), tr2.p3(), tr2.v2(), tr2.v3(), dt);
	intersectEdgeEdge(t[1], u[1], v[1], tr1.p1(), tr1.p3(), tr1.v1(), tr1.v3(), tr2.p2(), tr2.p3(), tr2.v2(), tr2.v3(), dt);
	intersectEdgeEdge(t[2], u[2], v[2], tr1.p2(), tr1.p3(), tr1.v2(), tr1.v3(), tr2.p2(), tr2.p3(), tr2.v2(), tr2.v3(), dt);

	// compute the point and so on
	core::collision::DetectionOutput *detectionOutput = new core::collision::DetectionOutput();
	detectionOutput->elem = std::pair<core::CollisionElementIterator, core::CollisionElementIterator>(tr1, tr2);

	return detectionOutput;
}

ContinuousTriangleIntersection::ContinuousTriangleIntersection(Triangle& t1, Triangle &t2):tr1(t1), tr2(t2),m_tolerance((SReal)(1e-6))
{
    m_tolmin = -m_tolerance;
    m_tolmax = (SReal)1.0 + m_tolerance;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
ContinuousTriangleIntersection::~ContinuousTriangleIntersection()
{

}

//////////////////////////////////////////////////////////////////////////////////////////////////
/// Method for calculating edge-edge collision
/// Given: edge (p1, p2) moving at velocity (v1, v2) intersecting with
///        edge (p3, p4) moving at velocity (v3, v4) 
///        dt is time step (in seconds)
/// Returns: 0 <= t <= 1 is the time of collision, actual time is t*dt
///          0 <= u, v <= 1 are the edge barycentric coordinates
/// Collision point: (1-u)*(p1 + t*dt*v1) + u*(p2 + t*dt*v2) = 
///		  or (1-v)*(p3 + t*dt*v3) + v*(p4 + t*dt*v4)
int ContinuousTriangleIntersection::intersectEdgeEdge (SReal& t, SReal& u, SReal& v,
													  const Vector3& p1, const Vector3& p2, 
													  const Vector3& v1, const Vector3& v2,
													  const Vector3& p3, const Vector3& p4,
													  const Vector3& v3, const Vector3& v4,
													  double dt)
{
    Vector3 d12, d13, d34, x12, x13, x34;
    SReal N1x, N1y, N1z, N2x, N2y, N2z, N3x, N3y, N3z;
    SReal C[4];
    Vector3 roots(-10,-10,-10);
    int numRoots;

    d12 = (v2 - v1)*dt;
    d34 = (v4 - v3)*dt;
    d13 = (v3 - v1)*dt;
    x12 = p2 - p1;
    x13 = p3 - p1;
    x34 = p4 - p3;

    /// Step1: Cross product x12(t) x x34(t)
    /// this represents the normal of triangle at t N(t)
    /// (x12 + t*dt*v12) x (x34 + t*dt*v34)
    /// Result: Quadratic equation in 't' in matrix form
    ///	   Vector N(t) { N3x + N2x*t + N1x*t*t,
    ///	     	         N3y + N2y*t + N1y*t*t,
    ///	     	         N3z + N2z*t + N1z*t*t}

    N1x = d12[1]*d34[2] - d12[2]*d34[1];
    N2x = x12[1]*d34[2] + d12[1]*x34[2] - x12[2]*d34[1] - x34[1]*d12[2];
    N3x = x12[1]*x34[2] - x12[2]*x34[1];

    N1y = d12[2]*d34[0] - d12[0]*d34[2];
    N2y = x12[2]*d34[0] + d12[2]*x34[0] - x12[0]*d34[2] - x34[2]*d12[0];
    N3y = x12[2]*x34[0] - x12[0]*x34[2];

    N1z = d12[0]*d34[1] - d12[1]*d34[0];
    N2z = x12[0]*d34[1] + d12[0]*x34[1] - x12[1]*d34[0] - x34[0]*d12[1];
    N3z = x12[0]*x34[1] - x12[1]*x34[0];

    /// Step 2: Compute the dot product with p01
    /// [N(t) . (p13 + t*dt*v13)] = 0
    /// compute coefficients of the resulting cubic equation
    /// C[0] + C[1]*t + C[2]*t*t + C[3]*t*t*t = 0

    C[3] = d13[0]*N1x + d13[1]*N1y + d13[2]*N1z;
    C[2] = x13[0]*N1x + d13[0]*N2x + x13[1]*N1y + d13[1]*N2y + x13[2]*N1z + d13[2]*N2z;
    C[1] = x13[0]*N2x + d13[0]*N3x + x13[1]*N2y + d13[1]*N3y + x13[2]*N2z + d13[2]*N3z;
    C[0] = x13[0]*N3x + x13[1]*N3y + x13[2]*N3z;

    /// Step 3: Determine roots of the cubic equation (for non-zero cubic term)
    if (C[3] < -EPSILON || C[3] > EPSILON)
        numRoots = solveCubic( roots, C );
    /// problem reduces to quadratic or linear (zero cubic term)
    else
        numRoots = solveQuadratic(roots[0], roots[1], C[2], C[1], C[0]);

    /// determing the valid root t [0,1] from upto three possible solutions
    bool isCorrectTime = checkValidRoots (t,
                                          numRoots,
                                          roots,
                                          m_tolmin,
                                          m_tolmax);

    if (!isCorrectTime)
        return 0;

    /// Step 4: determine exact points of collision within segments
    /// condition for intersection
    /// (1-u).x1(t) + u.x2(t) = (1-v).x3(t) + v.x4(t)
    x12 += d12*t;
    x34 += d34*t;
    x13 += d13*t;

    ///refer Bridson 2002 for calculating edge-edge

	SReal det = dot(x12, x12) * dot(x34, x34) - dot(x12, x34) * dot(x12, x34);

    if (det > -EPSILON && det < EPSILON)
    {
      std::cerr << "~ContinuousTriangleIntersection::intersectEdgeEdge: POSSIBLY PARALLEL EDGES, CANNOT FIND DETERMINANT" << std::endl;

        return 0;
    }

    SReal inv_det = (SReal)1.0/det;
    u = inv_det*(dot(x12, x13) * dot(x34,x34) - dot(x12, x34) * dot(x13, x34));

    if (u < m_tolmin || u > m_tolmax)
        return 0;

    v = inv_det*(dot(x12, x34) * dot(x12, x13) - dot(x12, x12) * dot(x13, x34) );

    if (v < m_tolmin || v > m_tolmax)
        return 0;

    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Method for calculating point-vertex collision
/// Given: Point p0 moving at velocity v0 intersecting with
///        Triangle (p1, p2, p3) moving at velocity (v1, v2, v3)
/// Returns: 0 <= t <= 1 is the time of collision, actual time is t*dt
///          0 <= u, v <= 1, u+v <= 1 are the triangle barycentric coordinates
/// Collision point: p0 + t*dt*v0 = 
///	             u*(p1 + t*dt*v1) + v*(p2 + t*dt*v2) + (1-u-v)*(p3 + t*dt*v3)
int ContinuousTriangleIntersection::intersectPointTriangle (SReal& t, SReal& u, SReal& v,
														    const Vector3& p0, const Vector3& v0,
														    const Vector3& p1, const Vector3& v1,
															const Vector3& p2, const Vector3& v2,
															const Vector3& p3, const Vector3& v3,
															double dt)
{
    Vector3 p12, v12, p13, v13, p01, v01;
    SReal N1x, N1y, N1z, N2x, N2y, N2z, N3x, N3y, N3z;
    SReal C[4];
    Vector3 roots(-10,-10,-10);

    /// Step 1: Compute the cross product p12 x p13
    /// this represents the normal of triangle at t N(t)
    /// (p12 + t*dt*v12) x (p13 + t*dt*v13)
    /// Result: Quadratic equation in 't' in matrix form
    ///	   Vector N(t) { N3x + N2x*t + N1x*t*t,
    ///	     	         N3y + N2y*t + N1y*t*t,
    ///	     	         N3z + N2z*t + N1z*t*t}

    p12 = p2 - p1;
    p13 = p3 - p1;
    p01 = p1 - p0;

    v12 = (v2 - v1) * dt;
    v13 = (v3 - v1) * dt;
    v01 = (v1 - v0) * dt;

    N1x = v12[1]*v13[2] - v12[2]*v13[1];
    N2x = p12[1]*v13[2] + v12[1]*p13[2] - p12[2]*v13[1] - p13[1]*v12[2];
    N3x = p12[1]*p13[2] - p12[2]*p13[1];

    N1y = v12[2]*v13[0] - v12[0]*v13[2];
    N2y = p12[2]*v13[0] + v12[2]*p13[0] - p12[0]*v13[2] - p13[2]*v12[0];
    N3y = p12[2]*p13[0] - p12[0]*p13[2];

    N1z = v12[0]*v13[1] - v12[1]*v13[0];
    N2z = p12[0]*v13[1] + v12[0]*p13[1] - p12[1]*v13[0] - p13[0]*v12[1];
    N3z = p12[0]*p13[1] - p12[1]*p13[0];

    /// Step 2: Compute the dot product with p01
    /// [N(t) . (p01 + t*v01)] = 0
    /// compute coefficients of the resulting cubic equation
    /// C[0] + C[1]*t + C[2]*t*t + C[3]*t*t*t = 0

    C[3] = v01[0]*N1x + v01[1]*N1y + v01[2]*N1z;
    C[2] = p01[0]*N1x + v01[0]*N2x + p01[1]*N1y + v01[1]*N2y + p01[2]*N1z + v01[2]*N2z;
    C[1] = p01[0]*N2x + v01[0]*N3x + p01[1]*N2y + v01[1]*N3y + p01[2]*N2z + v01[2]*N3z;
    C[0] = p01[0]*N3x + p01[1]*N3y + p01[2]*N3z;

    int numRoots;

    /// Step 3: determine roots of the cubic equation (for non-zero cubic term)
    if (C[3] < -EPSILON || C[3] > EPSILON)
        numRoots = solveCubic (roots, C);
    /// problem reduces to quadratic or linear (zero cubic term)
    else
        numRoots = solveQuadratic (roots[0], roots[1], C[2], C[1], C[0]);

    /// determing the valid root t [0,1] from upto three possible solutions
    bool isCorrectTime = checkValidRoots (t, numRoots, roots, m_tolmin, m_tolmax);

    if (!isCorrectTime)
        return 0;

    /// Step 4: determine exact points of collision within segments
    /// condition for intersection
    ///p0 + t*dt*v0 = u*(p1 + t*dt*v1) + v*(p2 + t*dt*v2) + (1-u-v)*(p3 + t*dt*v3)

    Vector3 x4 = p0 + v0*t*dt;
    Vector3 x1 = p1 + v1*t*dt;
    Vector3 x2 = p2 + v2*t*dt;
    Vector3 x3 = p3 + v3*t*dt;

    Vector3 x43 = x3 - x4;
    Vector3 x13 = x3 - x1;
    Vector3 x23 = x3 - x2;

    ///Ref. Bridson 2002
	SReal det = dot (x13, x13) * dot(x23, x23) - dot(x13, x23) * dot(x13, x23);

    if (det > -EPSILON && det < EPSILON)
    {
      std::cerr << "ContinousTriangleIntersection::intersectPointTriangle: POSSIBLY POINT IS PARALLEL TO PLANE, CANNOT FIND DETERMINANT" << std::endl;
        return 0;
    }

    ///find valid barycentric parameters
    SReal inv_det = (SReal)1./det;
	u = inv_det * ( dot (x13, x43) * dot(x23, x23) - dot(x13, x23) * dot (x23, x43) );

    if (u < m_tolmin || u > m_tolmax)
	{
		t = -10;
        return 0;
	}

	v = inv_det * ( dot(x13, x13) * dot(x23, x43) - dot(x13, x23) * dot(x13, x43) );

    if (v < m_tolmin || u + v > m_tolmax)
	{
		t = -10;
        return 0;
	}

    return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Method used internally
///Solves cubic equation: c[0] + c[1]*x + c[2]*x^2 + c[3]*x^3 = 0
///upto three possible solution, make sure c[3] is non-zero
///Ref Graphics Gems I, Schwarze, Jochen, Cubic and Quartic Roots, p. 404-407
int ContinuousTriangleIntersection::solveCubic (Vector3& s, SReal c[4] )
{
    int   i, num;
    SReal sub;
    SReal A, B, C;
    SReal sq_A, p, q;
    SReal cb_p, D;

    ///normal form: x^3 + Ax^2 + Bx + C = 0

    A = c[ 2 ] / c[ 3 ];
    B = c[ 1 ] / c[ 3 ];
    C = c[ 0 ] / c[ 3 ];

    ///  substitute x = y - A/3 to eliminate quadric term:
    ///  x^3 +px + q = 0

    sq_A = A * A;
    p = (SReal)1.0/3 * ((SReal)(-1.0/3) * sq_A + B);
    q = (SReal)1.0/2 * ((SReal)(2.0/27) * A * sq_A - (SReal)(1.0/3) * A * B + C);

    ///use Cardano's formula

    cb_p = p * p * p;
    D = q * q + cb_p;

    if (IsZero(D))
    {
        if (IsZero(q)) /// one triple solution
        {
            s[ 0 ] = 0;
            num = 1;
        }
        else /// one single and one SReal solution
        {
            SReal u = cbrt(-q);
            s[ 0 ] = 2 * u;
            s[ 1 ] = - u;
            num = 2;
        }
    }
    else if (D < 0) /// Casus irreducibilis: three SReal solutions
    {
        SReal phi = (SReal)1.0/3 * acos(-q / sqrt(-cb_p));
        SReal t = 2 * sqrt(-p);

        s[ 0 ] =   t * cos(phi);
        s[ 1 ] = - t * cos(phi + (SReal)R_PI / 3);
        s[ 2 ] = - t * cos(phi - (SReal)R_PI / 3);
        num = 3;
    }
    else /// one SReal solution
    {
        SReal sqrt_D = sqrt(D);
        SReal u = cbrt(sqrt_D - q);
        SReal v = - cbrt(sqrt_D + q);

        s[ 0 ] = u + v;
        num = 1;
    }

    /// resubstitute

    sub = (SReal)1.0/3 * A;

    for (i = 0; i < num; ++i)
        s[ i ] -= sub;

    return num;
}


///////////////////////////////////////////////////////////////////////////////////////////////////
/// Method used internally
/// Solves quadratic equation: a*X + b*X + c = 0
/// x1 = q/a; x2 = c/q;
/// where q = -1/2[b + sgn(b)*sqrt(b^2 -4ac)]
/// Ref. Numerical Recipes
int ContinuousTriangleIntersection::solveQuadratic (SReal& t1, SReal& t2, const SReal& a,
												   const SReal& b, const SReal& c)
{
    int numRoots=0;
    SReal delta = b*b - (SReal)4.0*a*c;
    if (delta < 0.0)
        return 0;

    SReal sqrt_delta = sqrt(delta);

    SReal sgn_b;
    if (b<0.0)
        sgn_b=-1.0;
    else
        sgn_b=1.0;

    SReal q = (SReal)(-0.5)*(b+sgn_b*sqrt_delta);

    if (a < -EPSILON || a > EPSILON)
    {
        t2 = q/a;
        numRoots++;
    }

    if (q < -EPSILON || q > EPSILON)
    {
        t1 = c/q;

        numRoots++;
    }
    //else
    //    serr << "ContinousTriangleIntersection::solveQuadratic: UNSOLVABLE QUADRATIC EQUATION" << sendl;


    return numRoots;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
/// Method used internally
/// method computes valid root within range of [rootMin, rootMax]
/// returns the smallest root if there are more than one valid root.
bool ContinuousTriangleIntersection::checkValidRoots(SReal& validRoot,
													const int& numRoots,
													const Vector3& roots,
													const SReal& rootMin,
													const SReal& rootMax)
{
    validRoot=-10;
    
    /// when numRoots equals 1
    if ((numRoots == 1) && checkRange (roots[0], rootMin, rootMax))
    {
        validRoot = roots[0];
        return true;
    }
    /// when numRoots equals 2
    else if (numRoots == 2)
    {
        if (checkRange(roots[0], rootMin, rootMax) && checkRange(roots[1], rootMin, rootMax))
        {
            validRoot = getMinVal(roots[0], roots[1]);
            return true;
        }
        else if (checkRange (roots[0], rootMin, rootMax))
        {
            validRoot = roots[0];
            return true;
        }
        else if (checkRange (roots[1], rootMin, rootMax))
        {
            validRoot = roots[1];
            return true;
        }
        else
            return false;
    }
    /// when numRoots equals 3
    else if (numRoots == 3)
    {
        if (checkRange (roots[0], rootMin, rootMax) &&
            checkRange (roots[1], rootMin, rootMax) &&
            checkRange (roots[2], rootMin, rootMax))
        {
            validRoot = getMinVal(roots[0], roots[1], roots[2]);
            return true;
        }
        else if (checkRange (roots[0], rootMin, rootMax) &&
                 checkRange (roots[1], rootMin, rootMax))
        {
            validRoot = getMinVal(roots[0], roots[1]);
            return true;
        }
        else if (checkRange (roots[1], rootMin, rootMax) &&
                 checkRange (roots[2], rootMin, rootMax))
        {
            validRoot = getMinVal(roots[1], roots[2]);
            return true;
        }
        else if (checkRange (roots[0], rootMin, rootMax) &&
                 checkRange (roots[2], rootMin, rootMax))
        {
            validRoot = getMinVal(roots[0], roots[2]);
            return true;
        }
        else if (checkRange (roots[0], rootMin, rootMax))
        {
            validRoot = roots[0];
            return true;
        }
        else if (checkRange (roots[1], rootMin, rootMax))
        {
            validRoot = roots[1];
            return true;
        }
        else if (checkRange (roots[2], rootMin, rootMax))
        {
            validRoot = roots[2];
            return true;
        }
        else
            return false;
    }
    else
        return false;
}

} // namespace collision

} // namespace component

} // namespace sofa

