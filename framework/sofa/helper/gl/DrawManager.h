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


#include <sofa/helper/gl/template.h>
#ifdef SOFA_GUI_QTOGREVIEWER
#include <OgreManualObject.h>
#endif

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
    enum MODEDISPLAY{OPENGL
#ifdef SOFA_GUI_QTOGREVIEWER
		     , OGRE
#endif
    };
    
    DrawManager();
    
    void drawPoints(const std::vector<Vector3> &points, float size,  const Vec<4,float> colour);
    void drawLines(const std::vector<Vector3> &points, float size, const Vec<4,float> colour);
    void drawTriangles(const std::vector<Vector3> &points, const Vec<4,float> colour);
    void drawTriangles(const std::vector<Vector3> &points, const Vector3 normal, const Vec<4,float> colour);


    void drawLines(const std::vector<Vector3> &points, const std::vector< defaulttype::Vec<2,int> > &index, float size, const Vec<4,float> colour);
    void drawTriangles(const std::vector<Vector3> &points, 
		       const std::vector< defaulttype::Vec<3,int> > &index, 
		       const std::vector<Vector3>  &normal,
		       const Vec<4,float> colour);
    void drawTriangles(const std::vector<Vector3> &points,
                       const std::vector<Vector3>  &normal,
                       const std::vector< Vec<4,float> > &colour);

    void drawTriangleStrip(const std::vector<Vector3> &points,
			   const std::vector<Vector3>  &normal,
			   const Vec<4,float> colour);
    void drawTriangleFan(const std::vector<Vector3> &points,
                         const std::vector<Vector3>  &normal,
                         const Vec<4,float> colour);

    void drawSpheres (const std::vector<Vector3> &points, const std::vector<float>& radius, const Vec<4,float> colour);
    void drawSpheres (const std::vector<Vector3> &points, float radius, const Vec<4,float> colour);
    //void drawFlatSpheres ( const std::vector<Vector3>& points, const std::vector<float>& radix, const std::vector< Vec<4,float> > colours); 
    void drawCone    (const Vector3& p1, const Vector3 &p2, float radius1, float radius2, const Vec<4,float> colour, int subdRadius=16);
    void drawCube    (const float& radius, const Vec<4,float>& colour, const int& subd=16); // Draw a cube of size one centered on the current point.
    void drawCylinder(const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd=16);
    void drawArrow   (const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd=16);
    void drawFrame   (const Vector3& position, const Quaternion &orientation, const Vec<3,float> &size);
    void drawPlus    (const float& radius, const Vec<4,float>& colour, const int& subd=16); // Draw a plus sign of size one centered on the current point.



    void addPoint(const Vector3 &p, const Vec<4,float> &c);
    void addPoint(const Vector3 &p, const Vector3 &n, const Vec<4,float> &c);
    void addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
		     const Vector3 &normal, const Vec<4,float> &c);
    void addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
                     const Vector3 &normal,
                     const Vec<4,float> &c1, const Vec<4,float> &c2, const Vec<4,float> &c3);


    void addSphere( const Vector3 &p, float radius);


    void setPolygonMode(int mode, bool wireframe);
    void setLightingEnabled(bool b);

    void setSystemDraw(MODEDISPLAY mode){SystemDraw=mode;}
    MODEDISPLAY getSystemDraw() const {return SystemDraw;}
#ifdef SOFA_GUI_QTOGREVIEWER
    void addOgreVertexPosition(const Vector3 &p);
    void addOgreVertexColour(const Vec<4,float> &p);
    void addOgreVertexNormal(const Vector3 &p);
    
    void setOgreObject(Ogre::ManualObject* o){ogreDraw=o;};
    void setOgreMaterial(Ogre::MaterialPtr s){currentMaterial=s;}
    void setSceneMgr(Ogre::SceneManager* s){mSceneMgr=s;}
#endif      

    void clear();
    void setMaterial(const Vec<4,float> &colour, std::string name=std::string());
    void resetMaterial(const Vec<4,float> &colour, std::string name=std::string());

    
  protected:
    MODEDISPLAY SystemDraw;
    bool lightEnabled;
    int polygonMode; //0: no cull, 1 front, 2 back
    bool wireframeEnabled;

#ifdef SOFA_GUI_QTOGREVIEWER
    Ogre::ManualObject *ogreDraw;
    Ogre::MaterialPtr currentMaterial;
    Ogre::SceneManager* mSceneMgr;



    //Basic shapes
    std::string sphereMeshName;

#endif
    static int materialName;
    static int meshName;
  };

} // namespace gl

} // namespace helper

} // namespace sofa

#endif
