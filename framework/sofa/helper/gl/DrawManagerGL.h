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
*                              SOFA :: Framework                              *
*                                                                             *
* Authors: M. Adam, J. Allard, B. Andre, P-J. Bensoussan, S. Cotin, C. Duriez,*
* H. Delingette, F. Falipou, F. Faure, S. Fonteneau, L. Heigeas, C. Mendoza,  *
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy
* JC. Lombardo, J.Esnault
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/


#ifndef DRAWMANAGERGL_H
#define DRAWMANAGERGL_H

#include "DrawManager.h"

namespace sofa
{

  namespace helper
  {

    namespace gl
    {
      
      using namespace defaulttype;

class SOFA_HELPER_API DrawManagerGL : public DrawManager 
{

public:
    DrawManagerGL();
    ~DrawManagerGL();
    
    // surcharge function from the DrawManager class
    
    virtual void drawPoints(const std::vector<Vector3> &points, float size,  const Vec<4,float> colour);
    
    virtual void drawLines(const std::vector<Vector3> &points, float size, const Vec<4,float> colour);
    virtual void drawLines(const std::vector<Vector3> &points, const std::vector< Vec<2,int> > &index, float size, const Vec<4,float> colour);
    
    virtual void drawTriangles(const std::vector<Vector3> &points, const Vec<4,float> colour);
    virtual void drawTriangles(const std::vector<Vector3> &points, const Vector3 normal, const Vec<4,float> colour);
    virtual void drawTriangles(const std::vector<Vector3> &points, 
		       const std::vector< Vec<3,int> > &index, 
		       const std::vector<Vector3>  &normal,
		       const Vec<4,float> colour);
    virtual void drawTriangles(const std::vector<Vector3> &points,
                       const std::vector<Vector3>  &normal,
                       const std::vector< Vec<4,float> > &colour);

    virtual void drawTriangleStrip(const std::vector<Vector3> &points,
			   const std::vector<Vector3>  &normal,
			   const Vec<4,float> colour);
    
    virtual void drawTriangleFan(const std::vector<Vector3> &points,
                         const std::vector<Vector3>  &normal,
                         const Vec<4,float> colour); 
    
    virtual void drawFrame(const Vector3& position, const Quaternion &orientation, const Vec<3,float> &size);

    virtual void drawSpheres (const std::vector<Vector3> &points, const std::vector<float>& radius, const Vec<4,float> colour);
    virtual void drawSpheres (const std::vector<Vector3> &points, float radius, const Vec<4,float> colour);
    
    //void drawFlatSpheres ( const std::vector<Vector3>& points, const std::vector<float>& radix, const std::vector< Vec<4,float> > colours);
 
    virtual void drawCone    (const Vector3& p1, const Vector3 &p2, float radius1, float radius2, const Vec<4,float> colour, int subd=16);
    
    virtual void drawCube    (const float& radius, const Vec<4,float>& colour, const int& subd=16); // Draw a cube of size one centered on the current point.
    
    virtual void drawCylinder(const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd=16);
   
    virtual void drawArrow   (const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd=16);

    virtual void drawPlus    (const float& radius, const Vec<4,float>& colour, const int& subd=16); // Draw a plus sign of size one centered on the current point.

    virtual void addPoint(const Vector3 &p, const Vec<4,float> &c);
    virtual void addPoint(const Vector3 &p, const Vector3 &n, const Vec<4,float> &c);
   
    virtual void addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
		     const Vector3 &normal, const Vec<4,float> &c);
    virtual void addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
                     const Vector3 &normal,
                     const Vec<4,float> &c1, const Vec<4,float> &c2, const Vec<4,float> &c3);

    virtual void addSphere( const Vector3 &p, float radius);
   
    virtual void clear();
   
    virtual void setMaterial(const Vec<4,float> &colour, std::string name=std::string());
   
    virtual void resetMaterial(const Vec<4,float> &colour, std::string name=std::string());

    virtual void pushMatrix() { glPushMatrix(); }
    virtual void popMatrix()  { glPopMatrix(); }
    virtual void multMatrix(float* glTransform ) { glMultMatrix(glTransform); }
    virtual void scale( float s ) { glScale(s,s,s); }

    
protected:
    // data member
    bool mLightEnabled;
    int  mPolygonMode;      //0: no cull, 1 front (CULL_CLOCKWISE), 2 back (CULL_ANTICLOCKWISE)
    bool mWireFrameEnabled;
    
public:
    // getter & setter
    virtual void setLightingEnabled(bool _isAnabled);
    
    bool getLightEnabled() {return mLightEnabled;}
    
    virtual void setPolygonMode(int _mode, bool _wireframe);
    
    bool getPolygonMode(){return mPolygonMode;}
    bool getWireFrameEnabled(){return mWireFrameEnabled;}
    
};

    }//namespace gl

  }//namespace helper

}//namespace sofa

#endif // DRAWMANAGERGL_H
