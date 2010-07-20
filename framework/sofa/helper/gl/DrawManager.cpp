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
#include <sofa/helper/gl/DrawManager.h>

#include <sofa/helper/system/gl.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/gl/Axis.h>

#ifdef SOFA_GUI_QTOGREVIEWER
#include <OgreRenderOperation.h>
#include <OgreMaterial.h> 
#include <Ogre.h>
#endif

namespace sofa
{

namespace helper
{

namespace gl
{
  DrawManager::DrawManager():SystemDraw(OPENGL)
#ifdef SOFA_GUI_QTOGREVIEWER
			    ,ogreDraw(NULL)
#endif
  {
    lightEnabled=false;
    wireframeEnabled=false;    
    polygonMode=1;
  };

  int DrawManager::materialName=0;
  int DrawManager::meshName=0;

  void DrawManager::drawPoints(const std::vector<Vector3> &points, float size, const Vec<4,float> colour=Vec<4,float>(1.0f,1.0f,1.0f,1.0f))
    {
	switch(SystemDraw)
	  {
	  case OPENGL: 
	    setMaterial(colour);
	    glPointSize(size);
	    glDisable(GL_LIGHTING);
	    glBegin(GL_POINTS);
	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;

	    setMaterial(colour);
	    currentMaterial->getTechnique(0)->setPointSize(size);   	    
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_POINT_LIST);
	  break;
#endif
	  }
	for (unsigned int i=0;i<points.size();++i)
	  {
	    addPoint(points[i], colour);
	  }

	switch(SystemDraw)
	  {
	  case OPENGL: 
	    glEnd();
	    if (lightEnabled) glEnable(GL_LIGHTING);
	    resetMaterial(colour);
	    glPointSize(1);
	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:	    

	    for (unsigned int i=0;i<points.size();++i)
	      ogreDraw->index(i);
            ogreDraw->end();
	  break;
#endif
	  }
    }

  void DrawManager::drawLines(const std::vector<Vector3> &points, float size, const Vec<4,float> colour)
  {
	switch(SystemDraw)
	  {
	  case OPENGL: 
	    setMaterial(colour);
	    glLineWidth(size);
	    glDisable(GL_LIGHTING);
	    glBegin(GL_LINES);
	    for (unsigned int i=0;i<points.size()/2;++i)
	      {	       
		addPoint(points[2*i]  , colour );
		addPoint(points[2*i+1], colour );
	      }	    
	    glEnd();
	    if (lightEnabled) glEnable(GL_LIGHTING);
	    resetMaterial(colour);
	    glLineWidth(1);

	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;

	    setMaterial(colour);
	    //Set Line Width: not possible with Ogre, with current version
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_LINE_LIST);

	    for (unsigned int i=0;i<points.size()/2;++i)
	      {
		addPoint(points[2*i], colour);
		addPoint(points[2*i+1], colour);
		ogreDraw->index(2*i);
		ogreDraw->index(2*i+1);
	      }
	    ogreDraw->end();
	  break;
#endif
	  }
  }



  void DrawManager::drawLines(const std::vector<Vector3> &points, const std::vector< defaulttype::Vec<2,int> > &index, float size, const Vec<4,float> colour=Vec<4,float>(1.0f,1.0f,1.0f,1.0f))
    {
	switch(SystemDraw)
	  {
	  case OPENGL: 
	    setMaterial(colour);
	    glLineWidth(size);
	    glDisable(GL_LIGHTING);
	    glBegin(GL_LINES);
	    for (unsigned int i=0;i<index.size();++i)
	      {	       
		addPoint(points[ index[i][0] ], colour );
		addPoint(points[ index[i][1] ], colour );
	      }	    
	    glEnd();
	    if (lightEnabled) glEnable(GL_LIGHTING);
	    resetMaterial(colour);
	    glLineWidth(1);

	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;

	    setMaterial(colour);
	    currentMaterial->getTechnique(0)->setPointSize(size); 
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_LINE_LIST);

	    for (unsigned int i=0;i<points.size();++i)
	      {
		addPoint(points[i], colour);
	      }

	    for (unsigned int i=0;i<index.size();++i)
	      {
		ogreDraw->index(index[i][0]);
		ogreDraw->index(index[i][1]);
	      }
	    ogreDraw->end();
	  break;
#endif
	  }
    }


  void DrawManager::drawTriangles(const std::vector<Vector3> &points, const Vec<4,float> colour)
  {
	switch(SystemDraw)
	  {
	  case OPENGL: 
	    setMaterial(colour);
	    glBegin(GL_TRIANGLES);
	    for (unsigned int i=0;i<points.size()/3;++i)
	    {
		const Vector3& a = points[ 3*i+0 ];
		const Vector3& b = points[ 3*i+1 ];
		const Vector3& c = points[ 3*i+2 ];
		Vector3 n = cross((b-a),(c-a));
		n.normalize();
		addTriangle(a,b,c,n,colour);
	    }
	    glEnd();
	    resetMaterial(colour);

	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;
	    setMaterial(colour);   
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

	    for (unsigned int i=0;i<points.size()/3;++i)
	    {	
		const Vector3& a = points[ 3*i+0 ];
		const Vector3& b = points[ 3*i+1 ];
		const Vector3& c = points[ 3*i+2 ];
		Vector3 n = cross((b-a),(c-a));
		n.normalize();
		addPoint(a,n,colour);
		addPoint(b,n,colour);
		addPoint(c,n,colour);
		ogreDraw->triangle(3*i, 3*i+1, 3*i+2);
	    }
	    ogreDraw->end();
	  break;
#endif
	  }
  }

  void DrawManager::drawTriangles(const std::vector<Vector3> &points, const Vector3 normal, const Vec<4,float> colour)
  {
	switch(SystemDraw)
	{
	  case OPENGL: 
	    setMaterial(colour);
	    glBegin(GL_TRIANGLES);
	    for (unsigned int i=0;i<points.size()/3;++i)
		addTriangle(points[ 3*i+0 ],points[ 3*i+1 ],points[ 3*i+2 ], normal, colour);
	    glEnd();
	    resetMaterial(colour);

	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;
	    setMaterial(colour);   
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

	    for (unsigned int i=0;i<points.size()/3;++i)
	      {	
		addPoint(points[ 3*i   ],colour);
		addPoint(points[ 3*i+1 ],colour);
		addPoint(points[ 3*i+2 ],colour);
		ogreDraw->triangle(3*i, 3*i+1, 3*i+2);
	      }
	    ogreDraw->end();
	  break;
#endif
	}
  }

  void DrawManager::drawTriangles(const std::vector<Vector3> &points, const std::vector< defaulttype::Vec<3,int> > &index, 
				  const std::vector<Vector3> &normal, const Vec<4,float> colour=Vec<4,float>(1.0f,1.0f,1.0f,1.0f))
    {
	switch(SystemDraw)
	  {
	  case OPENGL: 
	    setMaterial(colour);
	    glBegin(GL_TRIANGLES);
	    for (unsigned int i=0;i<index.size();++i)
	      {	      
		addTriangle(points[ index[i][0] ],points[ index[i][1] ],points[ index[i][2] ],normal[i],colour);
	      }	    
	    glEnd();
	    resetMaterial(colour);

	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;
	    setMaterial(colour);   
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

	    for (unsigned int i=0;i<index.size();++i)
	      {	
		addTriangle(points[ index[i][0] ],points[ index[i][1] ],points[ index[i][2] ],normal[i],colour);
		ogreDraw->triangle(index[i][0],index[i][1],index[i][2]);
	      }
	    ogreDraw->end();
	  break;
#endif
	  }
    }


  void DrawManager::drawTriangles(const std::vector<Vector3> &points,
                                  const std::vector<Vector3> &normal, const std::vector< Vec<4,float> > &colour)
    {
        const unsigned int nbTriangles=points.size()/3;
        bool computeNormals= (normal.size() != nbTriangles);
        if (nbTriangles == 0) return;
        switch(SystemDraw)
          {
          case OPENGL:
//            setMaterial(colour);
            glBegin(GL_TRIANGLES);
            for (unsigned int i=0;i<nbTriangles;++i)
              {
                if (!computeNormals)
                {
                    addTriangle(points[3*i+0],points[3*i+1],points[3*i+2],normal[i],
                                colour[3*i+0],colour[3*i+1],colour[3*i+2]);
                }
                else
                {
                    const Vector3& a = points[ 3*i+0 ];
                    const Vector3& b = points[ 3*i+1 ];
                    const Vector3& c = points[ 3*i+2 ];
                    Vector3 n = cross((b-a),(c-a));
                    n.normalize();

                    addPoint(a,n,colour[3*i+0]);
                    addPoint(b,n,colour[3*i+1]);
                    addPoint(c,n,colour[3*i+2]);

                }
              }
            glEnd();
//            resetMaterial(colour);

            break;
#ifdef SOFA_GUI_QTOGREVIEWER
          case OGRE:
            if (!ogreDraw) return;
//            setMaterial(colour);
            ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

            for (unsigned int i=0;i<nbTriangles;++i)
              {
                if (!computeNormals)
                {
                    addTriangle(points[3*i+0],points[3*i+1],points[3*i+2],normal[i],
                                colour[3*i+0],colour[3*i+1],colour[3*i+2]);
                }
                else
                {
                    const Vector3& a = points[ 3*i+0 ];
                    const Vector3& b = points[ 3*i+1 ];
                    const Vector3& c = points[ 3*i+2 ];
                    Vector3 n = cross((b-a),(c-a));
                    n.normalize();

                    addPoint(a,n,colour[3*i+0]);
                    addPoint(b,n,colour[3*i+1]);
                    addPoint(c,n,colour[3*i+2]);
                }
                ogreDraw->triangle(3*i+0,3*i+1,3*i+2);
              }
            ogreDraw->end();
          break;
#endif
          }
    }

  void DrawManager::drawTriangleStrip(const std::vector<Vector3> &points,
				      const std::vector<Vector3>  &normal,
				      const Vec<4,float> colour)
    {
	switch(SystemDraw)
	  {
	  case OPENGL: 
	    setMaterial(colour);
	    glBegin(GL_TRIANGLE_STRIP);
	    for (unsigned int i=0;i<normal.size();++i)
	      {	     
 		glNormalT(normal[i]); 
 		glVertexNv<3>(points[2*i].ptr());
 		glVertexNv<3>(points[2*i+1].ptr());
	      }	    
	    glEnd();
	    resetMaterial(colour);

	    break;
#ifdef SOFA_GUI_QTOGREVIEWER
	  case OGRE:
	    if (!ogreDraw) return;
	    setMaterial(colour);
	    ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_STRIP);

	    for (unsigned int i=0;i<normal.size();++i)
	      {	

		addOgreVertexPosition(points[2*i]);
		addOgreVertexNormal(normal[i]);
		addOgreVertexColour(colour);
		
		addOgreVertexPosition(points[2*i+1]);
		addOgreVertexNormal(normal[i]);
		addOgreVertexColour(colour);

		ogreDraw->index(2*i);
		ogreDraw->index(2*i+1);
	      }
	    ogreDraw->end();
	  break;
#endif
	  }
    }

  void DrawManager::drawTriangleFan(const std::vector<Vector3> &points,
                                      const std::vector<Vector3>  &normal,
                                      const Vec<4,float> colour)
    {
        if (points.size() < 3) return;
        switch(SystemDraw)
          {
          case OPENGL:
            setMaterial(colour);
            glBegin(GL_TRIANGLE_FAN);

            glNormalT(normal[0]);
            glVertexNv<3>(points[0].ptr());
            glVertexNv<3>(points[1].ptr());
            glVertexNv<3>(points[2].ptr());

            for (unsigned int i=3;i<points.size();++i)
            {
                glNormalT(normal[i]);
                glVertexNv<3>(points[i].ptr());
            }

            glEnd();
            resetMaterial(colour);

            break;
#ifdef SOFA_GUI_QTOGREVIEWER
          case OGRE:
            if (!ogreDraw) return;
            setMaterial(colour);
            ogreDraw->begin(currentMaterial->getName(), Ogre::RenderOperation::OT_TRIANGLE_FAN);


            addOgreVertexPosition(points[0]);
            addOgreVertexNormal(normal[0]);
            addOgreVertexColour(colour);

            addOgreVertexPosition(points[1]);
            addOgreVertexNormal(normal[0]);
            addOgreVertexColour(colour);

            addOgreVertexPosition(points[2]);
            addOgreVertexNormal(normal[0]);
            addOgreVertexColour(colour);

            ogreDraw->index(0);
            ogreDraw->index(1);
            ogreDraw->index(2);

            for (unsigned int i=3;i<points.size();++i)
            {
                addOgreVertexPosition(points[i]);
                addOgreVertexNormal(normal[i]);
                addOgreVertexColour(colour);
                ogreDraw->index(i);
            }

            ogreDraw->end();
          break;
#endif
          }
    }

  void DrawManager::drawSpheres(const std::vector<Vector3> &points, float radius, const Vec<4,float> colour)
    {
      setMaterial(colour);
      for (unsigned int i=0;i<points.size();++i)
	{
	  addSphere(points[i], radius);
	}
      resetMaterial(colour);
    }
  void DrawManager::drawSpheres(const std::vector<Vector3> &points, const std::vector<float>& radius, const Vec<4,float> colour)
    {      
      setMaterial(colour);
      for (unsigned int i=0;i<points.size();++i)
	{
	  addSphere(points[i], radius[i]);
	}
    resetMaterial(colour);
    }

  
  void DrawManager::drawCone(const Vector3& p1, const Vector3 &p2, float radius1, float radius2, const Vec<4,float> colour, int subd)
    {      
      Vector3 tmp = p2-p1;
      setMaterial(colour);
      /* create Vectors p and q, co-planar with the cylinder's cross-sectional disk */
      Vector3 p=tmp;
      if (fabs(p[0]) + fabs(p[1]) < 0.00001*tmp.norm())
	p[0] += 1.0;
      else
	p[2] += 1.0;
      Vector3 q;
      q = p.cross(tmp);
      p = tmp.cross(q);
      /* do the normalization outside the segment loop */
      p.normalize();
      q.normalize();
      
      int i2;
      float theta, st, ct;
      /* build the cylinder from rectangular subd */
      std::vector<Vector3> points;
      std::vector<Vec<4,int> > indices;
      std::vector<Vector3> normals;

      std::vector<Vector3> pointsCloseCylinder1;
      std::vector<Vector3> normalsCloseCylinder1;
      std::vector<Vector3> pointsCloseCylinder2;
      std::vector<Vector3> normalsCloseCylinder2;

      Vector3 dir=p1-p2; dir.normalize();
      pointsCloseCylinder1.push_back(p1);
      normalsCloseCylinder1.push_back(dir);
      pointsCloseCylinder2.push_back(p2);
      normalsCloseCylinder2.push_back(-dir);


      for (i2=0 ; i2<=subd ; i2++)
        {
          /* sweep out a circle */
          theta =  i2 * 2.0 * 3.14 / subd;
          st = sin(theta);
          ct = cos(theta);
          /* construct normal */
	  tmp = p*ct+q*st;
          /* set the normal for the two subseqent points */
	  normals.push_back(tmp);

          /* point on disk 1 */
          Vector3 w(p1);
          w += tmp*radius1;
	  points.push_back(w);
          pointsCloseCylinder1.push_back(w);
          normalsCloseCylinder1.push_back(dir);

          /* point on disk 2 */
          w=p2;
          w += tmp*radius2;
	  points.push_back(w);          
          pointsCloseCylinder2.push_back(w);
          normalsCloseCylinder2.push_back(-dir);
        }
      pointsCloseCylinder1.push_back(pointsCloseCylinder1[1]);
      normalsCloseCylinder1.push_back(normalsCloseCylinder1[1]);
      pointsCloseCylinder2.push_back(pointsCloseCylinder2[1]);
      normalsCloseCylinder2.push_back(normalsCloseCylinder2[1]);


      drawTriangleStrip(points, normals,colour);
      if (radius1 > 0) drawTriangleFan(pointsCloseCylinder1, normalsCloseCylinder1,colour);
      if (radius2 > 0) drawTriangleFan(pointsCloseCylinder2, normalsCloseCylinder2,colour);

      resetMaterial(colour);
    }

  void DrawManager::drawCylinder(const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour, int subd)
    {      
      drawCone( p1,p2,radius,radius,colour,subd);
    }

    void DrawManager::drawArrow(const Vector3& p1, const Vector3 &p2, float radius, const Vec<4,float> colour,  int subd)
    {
      
      Vector3 p3 = p1*.2+p2*.8;
      drawCylinder( p1,p3,radius,colour,subd);
      drawCone( p3,p2,radius*2.5,0,colour,subd);
    }

    void DrawManager::drawFrame   (const Vector3& position, const Quaternion &orientation, const Vec<3,float> &size)
    {
        setPolygonMode(0,false);
        switch(SystemDraw)
          {
          case OPENGL:
              helper::gl::Axis::draw(position, orientation, size);
              break;
#ifdef SOFA_GUI_QTOGREVIEWER
        case OGRE:

            setLightingEnabled(true);
            SReal matrix[16];
            orientation.writeOpenGlMatrix(matrix);

            Vector3 X(matrix[0*4+0], matrix[0*4+1],matrix[0*4+2]);
            Vector3 Y(matrix[1*4+0], matrix[1*4+1],matrix[1*4+2]);
            Vector3 Z(matrix[2*4+0], matrix[2*4+1],matrix[2*4+2]);

            drawArrow(position, position+X*size[0], 0.1*size[0], Vec<4,float>(1.0f,0.0f,0.0f,1.0f),16);
            drawArrow(position, position+Y*size[1], 0.1*size[1], Vec<4,float>(0.0f,1.0f,0.0f,1.0f),16);
            drawArrow(position, position+Z*size[2], 0.1*size[2], Vec<4,float>(0.0f,0.0f,1.0f,1.0f),16);

            setLightingEnabled(false);
            break;
#endif
        };


    }


  void DrawManager::addPoint(const Vector3 &p, const Vec<4,float> &c)
    {
      switch(SystemDraw)
	{
	case OPENGL: 
	  glColor4f(c[0],c[1],c[2],c[3]);
	  glVertexNv<3>(p.ptr());
	  break;
#ifdef SOFA_GUI_QTOGREVIEWER
	case OGRE: 
	  if (!ogreDraw) return;
	  addOgreVertexPosition(p);
	  addOgreVertexColour(c);
	  break;
#endif
	  };
      }

  void DrawManager::addPoint(const Vector3 &p, const Vector3 &n, const Vec<4,float> &c)
    {
      switch(SystemDraw)
	{
	case OPENGL: 
	  glColor4f(c[0],c[1],c[2],c[3]);
	  glNormalT(n);
	  glVertexNv<3>(p.ptr());
	  break;
#ifdef SOFA_GUI_QTOGREVIEWER
	case OGRE: 
	  if (!ogreDraw) return;
	  addOgreVertexPosition(p);
	  addOgreVertexNormal(p);
	  addOgreVertexColour(c);
	  break;
#endif
	  };
      }

  void DrawManager::addTriangle(const Vector3 &p1,const Vector3 &p2,const Vector3 &p3,
                                const Vector3 &normal,
                                const Vec<4,float> &c1, const Vec<4,float> &c2, const Vec<4,float> &c3)

  {      
      switch(SystemDraw)
        {
        case OPENGL:
          glNormalT(normal);
          glColor4fv(c1.ptr());
          glVertexNv<3>(p1.ptr());
          glColor4fv(c2.ptr());
          glVertexNv<3>(p2.ptr());
          glColor4fv(c3.ptr());
          glVertexNv<3>(p3.ptr());
          break;
#ifdef SOFA_GUI_QTOGREVIEWER
        case OGRE:
          if (!ogreDraw) return;
          addOgreVertexPosition(p1);
          addOgreVertexNormal(normal);
          addOgreVertexColour(c1);

          addOgreVertexPosition(p2);
          addOgreVertexNormal(normal);
          addOgreVertexColour(c2);
          addOgreVertexPosition(p3);
          addOgreVertexNormal(normal);
          addOgreVertexColour(c3);

          break;
#endif
          };
  };

  void DrawManager::addTriangle( const Vector3 &p1, const Vector3 &p2, const Vector3 &p3,
				 const Vector3 &normal, const  Vec<4,float> &c)
    {
      switch(SystemDraw)
	{
	case OPENGL: 
	  glNormalT(normal); 
	  glColor4fv(c.ptr());
	  glVertexNv<3>(p1.ptr());
	  glVertexNv<3>(p2.ptr());
	  glVertexNv<3>(p3.ptr());
	  break;
#ifdef SOFA_GUI_QTOGREVIEWER
	case OGRE: 
	  if (!ogreDraw) return;
	  addOgreVertexPosition(p1);
	  addOgreVertexNormal(normal);
	  addOgreVertexColour(c);

	  addOgreVertexPosition(p2);
	  addOgreVertexNormal(normal);
	  addOgreVertexColour(c);
	  addOgreVertexPosition(p3);
	  addOgreVertexNormal(normal);
	  addOgreVertexColour(c);

	  break;
#endif
	  };
      }


  void DrawManager::addSphere( const Vector3 &p, float radius)
  {
    switch(SystemDraw)
      {
      case OPENGL: 
	glPushMatrix();
	glTranslated(p[0], p[1], p[2]);
	glutSolidSphere(radius, 32, 16);
	glPopMatrix();
	break;
#ifdef SOFA_GUI_QTOGREVIEWER
      case OGRE:
	std::ostringstream s;
	s << "mesh[" << meshName++ <<"]";
  Ogre::Entity* sph = mSceneMgr->createEntity( s.str(), "mesh/ball.mesh" );//mSceneMgr->createEntity(s.str().c_str(), Ogre::SceneManager::PT_SPHERE );
  s.str("");
  s << "material[" << materialName-1 << "]" ;
  sph->setMaterialName(s.str());

  Ogre::SceneNode* node = mSceneMgr->getRootSceneNode()->createChildSceneNode();
  node->setScale(radius,radius,radius);
  node->setPosition(p[0],p[1],p[2]);
  node->attachObject(sph);
	break;         
#endif
      }
  }


#ifdef SOFA_GUI_QTOGREVIEWER
  void DrawManager::addOgreVertexPosition(const Vector3 &p)
  {
    ogreDraw->position(p[0],p[1],p[2]);
  }
  void DrawManager::addOgreVertexNormal(const Vector3 &p)
  {
    ogreDraw->normal(p[0],p[1],p[2]);
  }
  void DrawManager::addOgreVertexColour(const Vec<4,float> &p)
  {
    ogreDraw->colour(p[0],p[1],p[2],p[3]);
  }

#endif

  void DrawManager::setPolygonMode(int b, bool w)
  {
    polygonMode=b;
    wireframeEnabled=w;
    if (!polygonMode) 
      {
	if (wireframeEnabled) glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	else                  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
      }
    else if (polygonMode == 1)
      {
	if (wireframeEnabled) glPolygonMode(GL_FRONT, GL_LINE);
	else                  glPolygonMode(GL_FRONT, GL_FILL);
      }
     else if (polygonMode == 2)
     {
	if (wireframeEnabled) glPolygonMode(GL_BACK, GL_LINE);
	else                  glPolygonMode(GL_BACK, GL_FILL);
      }
  }


  void DrawManager::setLightingEnabled(bool b)
  {
    lightEnabled=b;
    if (lightEnabled) glEnable(GL_LIGHTING);
    else glDisable(GL_LIGHTING);
  }


  void DrawManager::setMaterial(const Vec<4,float> &colour,std::string 
#ifdef SOFA_GUI_QTOGREVIEWER
				name
#endif
				)
  {
    switch(SystemDraw)
      {
      case OPENGL:
		  {
	glColor4f(colour[0],colour[1],colour[2],colour[3]);
	glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, &colour[0]);
	static const float emissive[4] = { 0.0f, 0.0f, 0.0f, 0.0f};
	static const float specular[4] = { 1.0f, 1.0f, 1.0f, 1.0f};
	glMaterialfv (GL_FRONT_AND_BACK, GL_EMISSION, emissive);
	glMaterialfv (GL_FRONT_AND_BACK, GL_SPECULAR, specular);
	glMaterialf  (GL_FRONT_AND_BACK, GL_SHININESS, 20);
	if (colour[3] < 1)
	  {
	    glEnable(GL_BLEND);
	    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	    glDepthMask(0);
	  }
	else
	  {
	    glDisable(GL_BLEND);
	    glDepthMask(1);
	  }
	break;
		  }
#ifdef SOFA_GUI_QTOGREVIEWER
      case OGRE:
		  {
	//Get the Material
	if (name.empty())
	  {
	    std::ostringstream s;
	    s << "material[" << materialName++ << "]" ;
	    currentMaterial = Ogre::MaterialManager::getSingleton().create(s.str(), "General");
	  }
	else
	  currentMaterial = Ogre::MaterialManager::getSingleton().getByName(name);

	//Light
        currentMaterial->setLightingEnabled(lightEnabled);
	//Culling
	switch( polygonMode )
	  {
	  case 0:
	    currentMaterial->getTechnique(0)->setCullingMode(Ogre::CULL_NONE);
	    break;
	  case 1:
	    currentMaterial->getTechnique(0)->setCullingMode(Ogre::CULL_CLOCKWISE);
	    break;
	  case 2:
	    currentMaterial->getTechnique(0)->setCullingMode(Ogre::CULL_ANTICLOCKWISE);
	    break;
	  }



	//Blending
	if (colour[3] < 1) 
	  {
	    currentMaterial->setDepthWriteEnabled(false);
	    currentMaterial->getTechnique(0)->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
	    currentMaterial->setLightingEnabled(false); 
	    currentMaterial->setCullingMode(Ogre::CULL_NONE); 
	  }
	else
	  currentMaterial->setDepthWriteEnabled(true);

	//Shading
	currentMaterial->getTechnique(0)->setShadingMode(Ogre::SO_PHONG);
	  
	//Colour
 	currentMaterial->getTechnique(0)->getPass(0)->setDiffuse(Ogre::ColourValue(colour[0],colour[1],colour[2],colour[3]));
 	currentMaterial->getTechnique(0)->getPass(0)->setAmbient(Ogre::ColourValue(colour[0],colour[1],colour[2],colour[3]));
	currentMaterial->getTechnique(0)->getPass(0)->setSelfIllumination(Ogre::ColourValue(0,0,0,1));
	currentMaterial->getTechnique(0)->getPass(0)->setSpecular(Ogre::ColourValue(1,1,1,1)); 
	currentMaterial->getTechnique(0)->getPass(0)->setShininess(Ogre::Real(45));
	break;
		  }
#endif
      }
  }


  void DrawManager::resetMaterial(const Vec<4,float> &colour,std::string 
#ifdef SOFA_GUI_QTOGREVIEWER
                                  //name
#endif
				)
  {
    switch(SystemDraw)
      {
      case OPENGL:
	if (colour[3] < 1)
	  {
	    glDisable(GL_BLEND);
	    glDepthMask(1);
	  }
	break;
#ifdef SOFA_GUI_QTOGREVIEWER
      case OGRE:
	break;
#endif
      }
  }

  void DrawManager::clear()
  {
#ifdef SOFA_GUI_QTOGREVIEWER
    if (ogreDraw) ogreDraw->clear();
    for (int i=0;i<meshName;++i)
      {
	std::ostringstream s;
  s << "mesh[" << i <<"]" ;
  Ogre::SceneNode* n=(Ogre::SceneNode*)(mSceneMgr->getEntity(s.str())->getParentNode());
  mSceneMgr->destroyEntity(s.str());
  mSceneMgr->destroySceneNode(n);
      }

    for (int i=0;i<materialName;++i)
      {
	std::ostringstream s;
	s << "material[" << i <<"]" ;
	Ogre::MaterialManager::getSingleton().remove(s.str());
      }

    meshName=0;
    materialName=0;
#endif
  }
} // namespace gl

} // namespace helper

} // namespace sofa

