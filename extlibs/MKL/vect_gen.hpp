#include "debug.h"

template <class Vector>
ostream & vector_put(ostream &out, unsigned int length, Vector &a)
{
      out << '{';
      if(length>0) out << a[0];
      for(unsigned int i=1; i<length; i++)
	    out << ','<< a[i];
      out << '}' <<endl;
      return out;
}


template <class Vector>
void vector_null(const unsigned int length, Vector &a)
{
    for(unsigned int i=0; i<length; i++) a[i]=0;
}

template <class Vector>
void vector_null(Vector &a)
{
      vector_null(vector_length(a),a);
}

template <class Vector, class T>
void vector_fill(const unsigned int length, const T k, Vector &a)
{
    for(unsigned int i=0; i<length; i++) a[i]=k;
}

template <class Vector1, class Vector2>
void vector_copy(const unsigned int length, const Vector1 &a, Vector2 &b)
{
    for(unsigned int i=0; i<length; i++)
	    b[i]=a[i];
}

template <class Vector1, class Vector2>
void vector_copy(const Vector1 &a, Vector2 &b)
{
      REQUIRE(vector_length(a)==vector_length(b));
      vector_copy(vector_length(a),a,b);
}

template <class Vector1, class Vector2, class Vector3>
void vector_add(const unsigned int length, const Vector1 & a,const Vector2 & b, Vector3 &c)
{
    for(unsigned int i=0; i<length; i++)
	c[i]=a[i]+b[i];
}

template <class Vector1, class Vector2, class Vector3>
void vector_add(const Vector1 & a,const Vector2 & b, Vector3 &c)
{
      REQUIRE(vector_length(a) == vector_length(b));
      REQUIRE(vector_length(a) == vector_length(c));
      vector_add(vector_length(a),a,b,c);
}

template <class Vector1, class Vector2, class Vector3, class Vector4>
void vector_add3(const unsigned int length, const Vector1 & a,const Vector2 & b,const Vector3 &c, Vector4 &d)
{
      for(unsigned int i=0; i<length; i++)
	d[i]=a[i]+b[i]+c[i];
}

template <class Vector1, class Vector2, class Vector3, class Vector4>
void vector_add3(const Vector1 & a,const Vector2 & b,const  Vector3 &c, Vector4 &d)
{
       REQUIRE(vector_length(a) == vector_length(b));
       REQUIRE(vector_length(a) == vector_length(c));
       REQUIRE(vector_length(a) == vector_length(d));
       vector_add3(vector_length(a),a,b,c,d);
}

template <class Vector1, class T, class Vector3>
void vector_add_k(const unsigned int length, const Vector1 & a,const T & k, Vector3 &c)
{
    for(unsigned int i=0; i<length; i++)
	c[i]=a[i]+k;
}

template <class Vector1, class Vector2, class Vector3>
void vector_sub(const unsigned int length, const Vector1 & a,const Vector2 & b, Vector3 &c)
{
    for(unsigned int i=0; i<length; i++)
	c[i]=a[i]-b[i];
}

template <class Vector1, class Vector2, class Vector3>
void vector_sub(const Vector1 & a,const Vector2 & b, Vector3 &c)
{
      REQUIRE(vector_length(a) == vector_length(b));
      REQUIRE(vector_length(a) == vector_length(c));
      vector_sub(vector_length(a),a,b,c);
}

template <class Vector1, class T, class Vector2, class Vector3>
void vector_add_scale(const unsigned int length,
		      const Vector1 & a,
		      const T & k, const Vector2 & b,
		      Vector3 &c)
{
      for(unsigned int i=0; i<length; i++)
	    c[i]=a[i]+k*b[i];
}

template <class Vector1, class T, class Vector2, class Vector3>
void vector_add_scale(const Vector1 & a,
		      const T & k, const Vector2 & b,
		      Vector3 &c)
{
      REQUIRE(vector_length(a) == vector_length(b));
      REQUIRE(vector_length(a) == vector_length(c));
      vector_add_scale(vector_length(a),a,k,b,c);
      
}

template <class Vector1, class T1, class Vector2, class T2, class Vector3,class Vector4>
void vector_add3_scale(const unsigned int length,
		       const Vector1 & a,
		       const T1 & k1, const Vector2 & b,
		       const T2 & k2, const Vector3 & c,
		       Vector4 &d)
{
      for(unsigned int i=0; i<length; i++)
	    d[i]=a[i]+k1*b[i]+k2*c[i];
}

template <class Vector1, class T1, class Vector2, class T2, class Vector3, class Vector4>
void vector_add3_scale(const Vector1 & a,
		       const T1 & k1, const Vector2 & b,
		       const T2 & k2, const Vector3 & c,
		       Vector4 &d)
{
      REQUIRE(vector_length(a) == vector_length(b));
      REQUIRE(vector_length(a) == vector_length(c));
      REQUIRE(vector_length(a) == vector_length(d));
      vector_add3_scale(vector_length(a),a,k1,b,k2,c,d);
}


template <class Vector1, class T, class Vector3>
void vector_mul_k(const unsigned int length, const Vector1 & a,const T & k, Vector3 &c)
{
    for(unsigned int i=0; i<length; i++)
	c[i]=a[i]*k;
}
template <class Vector1, class T, class Vector3>
void vector_mul_k(const Vector1 & a,const T & k, Vector3 &c)
{
      REQUIRE(vector_length(a) == vector_length(c));
      vector_mul_k(vector_length(a),a,k,c);
}

template <class T, class Vector1, class Vector2>
void vector_scalar_product(const unsigned int length, const Vector1 & a, const Vector2 & b, T &c)
{
    c=0;
    for(unsigned int i=0; i<length; i++)
	c+=a[i]*b[i];
}

template <class T, class Vector1, class Vector2>
void vector_scalar_product(const Vector1 & a, const Vector2 & b, T &c)
{
      REQUIRE(vector_length(a)==vector_length(b));
      vector_scalar_product(vector_length(a),a,b,c);
}

template <class T, class Vector1, class Vector2>
T vector_scalar_product_T(const T* null, const Vector1 & a,const Vector2 & b)
{
      T t;
      vector_scalar_product(a,b,t);
      return t;
}


template <class Vector1, class Vector2, class Vector3>
void vector_cross_product(const Vector1 & a,const Vector2 & b, Vector3 &c)
{
    c[0]=a[1]*b[2]-a[2]*b[1];
    c[1]=a[2]*b[0]-a[0]*b[2];
    c[2]=a[0]*b[1]-a[1]*b[0];
}

template <class T, class Vector>
void vector_norm_square(const unsigned int size, const Vector &a, T &scalar)
{
    scalar=a[0]*a[0];
    for (unsigned int i=1;i<size;i++)
	scalar+=a[i]*a[i];
}

template <class T, class Vector>
void vector_norm(const unsigned int size, const Vector &a, T &scalar)
{
    vector_norm_square(size,a,scalar);
    scalar=sqrt(scalar);
}

template <class T, class Vector>
void vector_normalize_T(const T *p, const unsigned int size, Vector & a)
{
    T n;
    vector_norm(size, a, n);
    vector_mul_k(size,a,1/n,a);
}

template <class T, class Vector>
void vector_normalize_T(const T *p, Vector & a)
{
  vector_normalize_T(p,vector_length(a),a);
}

template <class Vector>
unsigned int vector_maximum(const unsigned int size, const Vector &v)
{
     int index=0;
     for(unsigned int i=1; i<size; i++) if(v[index]<v[i]) index=i;
     return index; 
}
