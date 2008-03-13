#ifndef SOFA_COMPONENT_COLLISION_CONTINUOUSTRIANGLEINTERSECTION_H
#define SOFA_COMPONENT_COLLISION_CONTINUOUSTRIANGLEINTERSECTION_H
#include <sofa/component/collision/Triangle.h>
#include <sofa/core/componentmodel/collision/DetectionOutput.h>
#include <math.h>

#define cbrt(x)     ((x) > 0.0 ? pow((double)(x), 1.0/3.0) : \
                    ((x) < 0.0 ? -pow((double)-(x), 1.0/3.0) : 0.0))
#define EPSILON 0.000001

#define EQN_EPS     1e-9
#define	IsZero(x)	((x) > -EQN_EPS && (x) < EQN_EPS)

namespace sofa
{

namespace component
{

namespace collision
{

using namespace collision;

class ContinuousTriangleIntersection
{
private:
	Triangle &tr1, &tr2;
	double m_tolerance, m_tolmin, m_tolmax;

	int intersectPointTriangle (double& t, double& u, double& v,
                                const Vector3& p0, const Vector3& v0,
                                const Vector3& p1, const Vector3& v1,
                                const Vector3& p2, const Vector3& v2,
                                const Vector3& p3, const Vector3& v3,
                                double dt);

    int intersectEdgeEdge(double& t, double& u, double& v,
                           const Vector3& p1, const Vector3& p2,
                           const Vector3& v1, const Vector3& v2,
                           const Vector3& p3, const Vector3& p4,
                           const Vector3& v3, const Vector3& v4,
                           double dt);

	int solveCubic(Vector3& s, double c[4]);
    
    int solveQuadratic (double& t1, 
						double& t2, 
						const double& a, 
						const double& b, 
						const double& c);
    
    bool checkValidRoots (double& validRoot,
                          const int& numRoots,
                          const Vector3& roots,
                          const double& rootMin,
                          const double& rootMax);

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns true of value lies within the range
    bool checkRange (const double& val,
                     const double& min,
                     const double& max)
    {
        return ((val >= min) && (val <= max));
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns the smallest of the two values
    inline double getMinVal (const double& val0, const double& val1)
    {
		return (val0 < val1) ? val0 : val1;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////
    /// returns the smallest of the three values
   double getMinVal(const double &val0,
							const double &val1,
							const double &val2)
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
    void setTolerance(const double &tol)
    {
        m_tolerance = tol;
		m_tolmin = -m_tolerance;
		m_tolmax = 1.0+m_tolerance;
    };

public:
	ContinuousTriangleIntersection (Triangle& t1, Triangle& t2);
	~ContinuousTriangleIntersection(void);
	core::componentmodel::collision::DetectionOutput* computeDetectionOutput (void);
	bool isCollision(void);
};

} // namespace collision

} // namespace component

} // namespace sofa


#endif /* _CONTINUOUSTRIANGLEINTERSECTION_H_ */
