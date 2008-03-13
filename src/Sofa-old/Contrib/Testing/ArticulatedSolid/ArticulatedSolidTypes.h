//
// C++ Interface: ArticulatedSolidTypes
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef Sofa_ComponentsArticulatedSolidTypes_h
#define Sofa_ComponentsArticulatedSolidTypes_h

#include <Sofa-old/Components/Common/Vec.h>
#include <Sofa-old/Components/Common/Quat.h>
#include <Sofa-old/Components/Common/Mat.h>
#include <Sofa-old/Components/Common/fixed_array.h>
#include <iostream>

namespace Sofa
{

namespace Components
{



/**
Base types for the ArticulatedSolid: position, orientation, velocity, angular velocity, etc.
 
@author The SOFA team
*/
template< class R=float >
class ArticulatedSolidTypes
{
public:
        typedef R Real;
        typedef Common::Vec<3,Real> Vec;
        typedef Common::Quater<Real> Rot;
        typedef Common::Mat<3,3,Real> Mat;


        static Mat dyad( const Vec& u, const Vec& v )
        {
                Mat m;
                for( int i=0; i<3; i++ )
                        for( int j=0; j<3; j++ )
                                m[i][j] = u[i]*v[j];
                return m;
        }


        /** A spatial vector */
        class SpatialVector
        {
        public:
                Vec lineVec;
                Vec freeVec;
                void clear()
                {
                        lineVec = freeVec = Vec(0,0,0);
                }
                SpatialVector()
                {}
                SpatialVector( const Vec& l, const Vec& f ):lineVec(l),freeVec(f)
                {}
                SpatialVector& operator += (const SpatialVector& v)
                {
                        lineVec += v.lineVec;
                        freeVec += v.freeVec;
                        return *this;
                }
                SpatialVector operator * ( Real a ) const
                {
                        return SpatialVector( lineVec *a, freeVec * a);
                }
                SpatialVector& operator *= ( Real a )
                {
                        lineVec *=a, freeVec *= a;
                        return *this;
                }
                SpatialVector operator + ( const SpatialVector& v ) const
                {
                        return SpatialVector(lineVec+v.lineVec,freeVec+v.freeVec);
                }
                SpatialVector operator - ( const SpatialVector& v ) const
                {
                        return SpatialVector(lineVec-v.lineVec,freeVec-v.freeVec);
                }
                SpatialVector operator - ( ) const
                {
                        return SpatialVector(-lineVec,-freeVec);
                }
                /// Spatial dot product (cross terms)
                Real operator * ( const SpatialVector& v ) const
                {
                        return lineVec * v.freeVec + freeVec * v.lineVec;
                }
                /// Spatial cross product
                SpatialVector cross( const SpatialVector& v ) const
                {
                        return SpatialVector(
                                       Common::cross(lineVec,v.lineVec),
                                       Common::cross(freeVec,v.lineVec) + Common::cross(lineVec,v.freeVec)
                               );
                }
                void print( std::ostream& out ) // Because operator << does not work (why?)
                {
                        out<<lineVec<<" "<<freeVec;
                }
        };

        /** Define a frame (the child) whith respect to another (the parent).
        Internal data represents the orientation of the child wrt the parent, BUT the translation vector represents the origin of the parent with respect to the child. For example, the coordinates M_p of point M in parent given the coordinates M_c of the same point in child are given by: M_p = orientation * ( M_c - origin ). This is so to obey Featherstone's conventions. Use method setTranslationRotation( const Vec& t, const Rot& q ) to model the Transform the standard way.


        */
        class Transform
        {
        public:
                Transform()
                {}
                /// Define using Featherstone's conventions
                Transform( const Rot& q, const Vec& o ):orientation_(q),origin_(o)
                {}
                /// Define given the origin of the child wrt the parent and the orientation of the child wrt the parent (i.e. standard way)
                void setTranslationRotation( const Vec& t, const Rot& q )
                {
                        orientation_ =q, origin_ = -(q.rotate(t));
                }
                /// Define as a given SpatialVector integrated during one second
                Transform( const SpatialVector& v )
                {
                        origin_ = v.freeVec;
                        orientation_ = Rot::createFromRotationVector( v.lineVec );
                }
                const Vec& getOrigin() const
                {
                        return origin_;
                }
                const Rot& getOrientation() const
                {
                        return orientation_;
                }
		Mat getRotationMatrix() const {
		    Mat m;
		    m[0][0] = (1.0 - 2.0 * (orientation_[1] * orientation_[1] + orientation_[2] * orientation_[2]));
		    m[0][1] = (2.0 * (orientation_[0] * orientation_[1] - orientation_[2] * orientation_[3]));
		    m[0][2] = (2.0 * (orientation_[2] * orientation_[0] + orientation_[1] * orientation_[3]));

		    m[1][0] = (2.0 * (orientation_[0] * orientation_[1] + orientation_[2] * orientation_[3]));
		    m[1][1] = (1.0 - 2.0 * (orientation_[2] * orientation_[2] + orientation_[0] * orientation_[0]));
		    m[1][2] = (2.0 * (orientation_[1] * orientation_[2] - orientation_[0] * orientation_[3]));

		    m[2][0] = (2.0 * (orientation_[2] * orientation_[0] - orientation_[1] * orientation_[3]));
		    m[2][1] = (2.0 * (orientation_[1] * orientation_[2] + orientation_[0] * orientation_[3]));
		    m[2][2] = (1.0 - 2.0 * (orientation_[1] * orientation_[1] + orientation_[0] * orientation_[0]));
		    return m;
		}		    
/*		Mat getRotationMatrix() const {
		    Mat m;
		    m[0][0] = (1.0 - 2.0 * (orientation_[1] * orientation_[1] + orientation_[2] * orientation_[2]));
		    m[1][0] = (2.0 * (orientation_[0] * orientation_[1] - orientation_[2] * orientation_[3]));
		    m[2][0] = (2.0 * (orientation_[2] * orientation_[0] + orientation_[1] * orientation_[3]));

		    m[0][1] = (2.0 * (orientation_[0] * orientation_[1] + orientation_[2] * orientation_[3]));
		    m[1][1] = (1.0 - 2.0 * (orientation_[2] * orientation_[2] + orientation_[0] * orientation_[0]));
		    m[2][1] = (2.0 * (orientation_[1] * orientation_[2] - orientation_[0] * orientation_[3]));

		    m[0][2] = (2.0 * (orientation_[2] * orientation_[0] - orientation_[1] * orientation_[3]));
		    m[1][2] = (2.0 * (orientation_[1] * orientation_[2] + orientation_[0] * orientation_[3]));
		    m[2][2] = (1.0 - 2.0 * (orientation_[1] * orientation_[1] + orientation_[0] * orientation_[0]));
		    return m;
		}		    */
		void clear()
                {
                        orientation_.clear();
                        origin_=Vec(0,0,0);
                }
                Transform operator * (const Transform& f2) const
                {
                        return Transform(  orientation_ * f2.getOrientation(), f2.getOrigin() + f2.getOrientation().inverseRotate(origin_)) ; 
                }
                Transform& operator *= (const Transform& f2)
                {
                        orientation_ *= f2.orientation;
                        origin_ = f2.getOrigin() + f2.getOrientation().inverseRotate(origin_);
                        return (*this);
                }
                SpatialVector operator * (const SpatialVector& sv ) const
                {
                        return SpatialVector(
                                       orientation_.rotate(sv.lineVec),
                                       orientation_.rotate(sv.freeVec - cross( origin_, sv.lineVec) )
                               );
                }
                Transform& operator += (const SpatialVector& v)
                {
                        this *= Transform(v);
                        return *this;
                }
                Transform inversed() const
                {
                        return Transform( orientation_.inverse(), -(orientation_.inverse().rotate(origin_)) );
                }
                void writeOpenGlMatrix( double *m ) const
                {
                        orientation_.writeOpenGlMatrix(m);
                        Vec t = -orientation_.rotate(origin_);
                        m[12] = t[0];
                        m[13] = t[1];
                        m[14] = t[2];
                }
		void print( std::ostream& out ) // Because operator << does not work (why?)
		{
		    out<<"t= "<<origin_<<std::endl;
		    out<<"E= "<<orientation_<<std::endl;
		}

        protected:
                Rot orientation_; ///< child wrt parent
                Vec origin_;  ///< parent wrt child

        };

        typedef Common::Vec<6,Real> DOF;

        static Vec mult( const Mat& m, const Vec& v )
        {
                Vec r;
                for( int i=0; i<3; ++i ) {
                        r[i]=0;
                        for( int j=0; j<3; ++j )
                                r[i]+=m[i][j] * v[j];
                }
                return r;
        }

        static Vec multTrans( const Mat& m, const Vec& v )
        {
                Vec r;
                for( int i=0; i<3; ++i ) {
                        r[i]=0;
                        for( int j=0; j<3; ++j )
                                r[i]+=m[j][i] * v[j];
                }
                return r;
        }

	/// Cross product matrix of a vector
	static Mat crossM( const Vec& v )
        {
                Mat m;
                m[0][0]=0;
                m[0][1]=-v[2];
                m[0][2]= v[1];
                m[1][0]= v[2];
                m[1][1]=0;
                m[1][2]=-v[0];
                m[2][0]=-v[1];
                m[2][1]= v[0];
                m[2][2]=0;
                return m;
        }

        class RigidInertia
        {
        public:
                Real m;
                Vec h;
                Mat I;
                RigidInertia()
                {}
                RigidInertia( Real m, const Vec& h, const Mat& I ):m(m),h(h),I(I)
                {}
                SpatialVector operator * (const SpatialVector& v ) const
                {
                        return SpatialVector(
                                       cross(v.lineVec,h)+v.freeVec*m,
                                       mult(I,v.lineVec) + cross( h, v.freeVec )
                               );
                }
                RigidInertia operator * ( const Transform& t ) const
                {
		    Vec h_mr = h - t.getOrigin() * m;
		    Mat E = t.getRotationMatrix();
		    return RigidInertia( 
			    m, E*h_mr, 
		    E*(I+crossM(t.getOrigin())*crossM(h)+crossM(h_mr)*crossM(t.getOrigin()))*(E.transposed()) );
                }
                void print( std::ostream& out ) const // Because operator << does not work (why?)
                {
                        out<<"I= "<<I<<std::endl;
                        out<<"h= "<<h<<std::endl;
                        out<<"m= "<<m<<std::endl;
                }

        };

        class ArticulatedInertia
        {
        public:
	    Mat M;
	    Mat H;
		Mat I;
		ArticulatedInertia()
                {}
		ArticulatedInertia( const Mat& M, const Mat& H, const Mat& I ):M(M),H(H),I(I)
                {}
                SpatialVector operator * (const SpatialVector& v ) const
                {
                        return SpatialVector(
                                       multTrans(H,v.lineVec) + mult(M,v.freeVec),
                                       mult(I,v.lineVec) + mult(H,v.freeVec)
                               );

                }
                ArticulatedInertia operator * ( Real r ) const 
                {
		    return ArticulatedInertia( M*r, H*r, I*r );

                }
                ArticulatedInertia& operator = (const RigidInertia& Ri )
                 {
//                         H[0][0]=0;
//                         H[0][1]=-Ri.h[2];
//                         H[0][2]= Ri.h[1];
//                         H[1][0]= Ri.h[2];
//                         H[1][1]=0;
//                         H[1][2]=-Ri.h[0];
//                         H[2][0]=-Ri.h[1];
//                         H[2][1]= Ri.h[0];
//                         H[2][2]=0;
		     H = crossM( Ri.h );
			
                        for( int i=0; i<3; i++ )
                                for( int j=0; j<3; j++ )
                                        M[i][j]= i==j ? Ri.m : 0;

                        I=Ri.I;
                        return *this;
                }
                ArticulatedInertia& operator += (const ArticulatedInertia& Ai )
                {
                        H += Ai.H;
                        M += Ai.M;
                        I += Ai.I;
                        return *this;
                }
		ArticulatedInertia operator + (const ArticulatedInertia& Ai ) const
		{
		    return ArticulatedInertia(M+Ai.M, H+Ai.H, I+Ai.I);
		}
		ArticulatedInertia operator - (const ArticulatedInertia& Ai ) const
		{
		    return ArticulatedInertia(M-Ai.M, H-Ai.H, I-Ai.I);
		}
		void print( std::ostream& out ) const // Because operator << does not work (why?)
                {
                        out<<"I= "<<I<<std::endl;
                        out<<"H= "<<H<<std::endl;
                        out<<"M= "<<M<<std::endl;
                }
        };
	
// 	typedef Transform VecCoord;
// 	typedef SpatialVector VecDeriv;
// 	typedef Transform Coord;
// 	typedef SpatialVector Deriv;


        static ArticulatedInertia dyad ( const SpatialVector& u, const SpatialVector& v )
        {
	    //return ArticulatedInertia(dyad(u.lineVec, v.freeVec), dyad(u.freeVec, v.freeVec),  dyad(u.freeVec, v.lineVec));
	    return ArticulatedInertia(dyad(u.lineVec, v.lineVec), dyad(u.freeVec, v.lineVec),  dyad(u.freeVec, v.freeVec));
	}

};


}//Components

}//Sofa


// Why does this not work ?????
template<class R>
std::ostream& operator << (std::ostream& out, const typename Sofa::Components::ArticulatedSolidTypes<R>::SpatialVector& sv )
{
        out << sv.lineVec <<" "<< sv.freeVec;
        return out;
}

#endif

