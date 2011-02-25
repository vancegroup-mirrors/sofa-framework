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
* M. Nesme, P. Neumann, J-P. de la Plata Alcade, F. Poyer and F. Roy          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_HELPER_GL_DRAWMANAGER_H
#define SOFA_HELPER_GL_DRAWMANAGER_H

#include <sofa/helper/helper.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/Quat.h>

#include <vector>

#include "template.h"

namespace sofa
{

  namespace helper
  {

    namespace gl
    {
  
      using namespace defaulttype;

  class SOFA_HELPER_API DrawManager
  {
  public:
             DrawManager() {}
    virtual ~DrawManager() {}
    
    virtual void drawPoints(const std::vector<Vector3> &points, float size,  const Vec<4,float> colour) = 0 ;
    
    virtual void drawLines(const std::vector<Vector3> &points, float size, const Vec<4,float> colour) = 0 ;
    virtual void drawLines(const std::vector<Vector3> &points, const std::vector< Vec<2,int> > &index, float size, const Vec<4,float> colour) = 0 ;
    
    virtual void drawTriangles(const std::vector<Vector3> &points, const Vec<4,float> colour) = 0 ;
    virtual void drawTriangles(const std::vector<Vector3> &points, const Vector3 normal, const Vec<4,float> colour) = 0 ;
    virtual void drawTriangles(const std::vector<Vector3> &points, 
		       const std::vector< Vec<3,int> > &index, 
		       const std::vector<Vector3>  &normal,
		       const Vec<4,float> colour) = 0 ;
    virtual void drawTriangles(const std::vector<Vector3> &points,
                       const std::vector<Vector3>  &normal,
                       const std::vector< Vec<4,float> > &colour) = 0 ;

    virtual void drawTriangleStrip(const std::vector<Vector3> &points,
			   const std::vector<Vector3>  &normal,
			   const Vec<4,float> colour) = 0 ;
			   
    virtual void drawTriangleFan(const std::vector<Vector3> &points,
                         const std::vector<Vector3>  &normal,
                         const Vec<4,float> colour) = 0 ;
    
    virtual void drawFrame   (const Vector3& position, const Quaternion &orientation, const Vec<3,float> &size) = 0 ;

    virtual void drawSpheres (const std::vector<Vector3> &points, const std::vector<float>& radius, const Vec<4,float> colour) = 0;
    virtual void drawSpheres (const std::vector<Vector3> &points, float radius, const Vec<4,float> colour) = 0 ;
    
    //void drawFlatSpheres ( const std::vector<Vector3>& points, const std::vector<float>& radix, const std::vector< Vec<4,float> > colours) = 0 ;
 
    virtual void drawCone    (const Vector3& p1, const Vector3 &p2, float radius1, float radius2, const Vec<4,float> colour, int subd=16) = 0 ;
    //virtual void drawCone    (const Vector3& p1, const Vector3 &p2, float radius1, float radius2, const Vec<4,float> colour, int subdRadius=16) = 0 ;
    
    virtual void drawCube    (const float& radius, const Vec<4,float>& colour, const int& subd=16) = 0 ; // Draw a cube of size one centered on the current point.

    virtual void drawCylinder(const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd=16) = 0 ;
   
    virtual void drawArrow   (const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd=16) = 0 ;

    virtual void drawPlus    (const float& radius, const Vec<4,float>& colour, const int& subd=16) = 0 ; // Draw a plus sign of size one centered on the current point.
    
    virtual void addPoint(const Vector3 &p, const Vec<4,float> &c) = 0 ;
    virtual void addPoint(const Vector3 &p, const Vector3 &n, const Vec<4,float> &c) = 0 ;
    
    virtual void addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
		     const Vector3 &normal, const Vec<4,float> &c) = 0 ;
    virtual void addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
                     const Vector3 &normal,
                     const Vec<4,float> &c1, const Vec<4,float> &c2, const Vec<4,float> &c3) = 0 ;

    virtual void addSphere( const Vector3 &p, float radius) = 0 ;
    
    virtual void clear() = 0 ;
    
    virtual void setMaterial(const Vec<4,float> &colour, std::string name=std::string()) = 0 ;
    
    virtual void resetMaterial(const Vec<4,float> &colour, std::string name=std::string()) = 0 ;

    virtual void setPolygonMode(int _mode, bool _wireframe) = 0 ;

    virtual void setLightingEnabled(bool _isAnabled) = 0 ;  

    virtual void pushMatrix() = 0;
    virtual void popMatrix() =  0;
    virtual void multMatrix(float*  ) = 0;
    virtual void scale(float ) = 0;

        };

    } // namespace gl

  } // namespace helper

} // namespace sofa

#endif //SOFA_HELPER_GL_DRAWMANAGER_H
