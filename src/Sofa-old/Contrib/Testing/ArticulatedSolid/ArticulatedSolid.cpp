//
// C++ Implementation: ArticulatedSolid
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ArticulatedSolid.inl"
#include "ArticulatedSolidTypes.h"
#include "ArticulatedSolidNode.h"
#include "ArticulatedSolidRevolute.h"
#include <iostream>
#include <sstream>
using std::cout;
using std::cerr;
using std::endl;
using std::istringstream;

namespace Sofa
{

namespace Components
{

template class ArticulatedSolid< ArticulatedSolidTypes<float> >
;

// typedef ArticulatedSolid< ArticulatedSolidTypes<double> > ArticulatedSolidd;
// template class ArticulatedSolid< ArticulatedSolidTypes<double> >
// ;

}//Components
}//Sofa

namespace Sofa
{
namespace Components
{
    typedef ArticulatedSolidEulerXYZ< ArticulatedSolidTypes<float> > ArticulatedSolidEulerXYZf;
    
namespace Common
{
/// Construct an ArticulatedSolid object from a XML node.
//void create(ArticulatedSolidf*& obj, XML::Node<Abstract::BehaviorModel>* arg)
void create(ArticulatedSolidf*& obj, XML::Node<Core::DynamicModel>* arg)
{

        
    
    // create the solid of the appropriate type
        if( const char* jointType = arg->getAttribute("jointType") ) {
                cout<<"create(ArticulatedSolidf*& obj, XML::Node<Abstract::BehaviorModel>* arg), jointType = "<< jointType << endl;
                std::string jt(jointType);
                if( jt=="revolute" ) {
                        obj = new ArticulatedSolidRevolute<ArticulatedSolidTypes<float> >;
                } else
                        cerr<<"create(ArticulatedSolidf*& obj, XML::Node<Core::DynamicModel>* arg) , unknown joint type "<< jt << endl;
        } else {
                cerr<<"create(ArticulatedSolidf*& obj, XML::Node<Core::DynamicModel>* arg) , no jointType given"<< endl;
                return;
        }

	ArticulatedSolidf::Vec jointCenter(0,0,0);
	ArticulatedSolidf::Vec jointAxis(1,0,0); // rotate the joint frame
	ArticulatedSolidf::Real jointAngle = 0;
	ArticulatedSolidf::Real xx=1,yy=1,zz=1,xy=0,yz=0,zx=0;
	ArticulatedSolidf::Real m = 1;
	ArticulatedSolidf::Vec massCenter(0,0,0);
	
        // set the ArticulatedSolid fields
        if( const char* s = arg->getAttribute("jointCenter") ) {
                istringstream in(s);
                in >> jointCenter;
        }
	if( const char* s = arg->getAttribute("jointOrientation") ) {
		istringstream in(s);
		in >> jointAxis;
		in >> jointAngle;
	}
	if( const char* s = arg->getAttribute("inertia") ) {
		istringstream in(s);
		in >>xx >>yy >>zz >>xy >>yz >>zx ;
	}
	if( const char* s = arg->getAttribute("mass") ) {
		istringstream in(s);
		in >>m ;
	}
	if( const char* s = arg->getAttribute("massCenter") ) {
	    istringstream in(s);
	    in >>massCenter ;
	}
	if( const char* s = arg->getAttribute("dof") ) {
	    ArticulatedSolidEulerXYZf* ae = dynamic_cast<ArticulatedSolidEulerXYZf*>(obj);
	    if( ae ){
		istringstream in(s);
		ArticulatedSolidEulerXYZf::Coord d;
		in >>d ;
		ae->setCoordinates(d);
	    }
	    else cerr<<"create(ArticulatedSolidf*& obj, XML::Node<Core::DynamicModel>* arg): can not set dof values of a non-EulerXYZ solid"<<endl;
	}
	if( const char* s = arg->getAttribute("dofVel") ) {
	    ArticulatedSolidEulerXYZf* ae = dynamic_cast<ArticulatedSolidEulerXYZf*>(obj);
	    if( ae ){
		istringstream in(s);
		ArticulatedSolidEulerXYZf::Deriv d;
		in >>d ;
		ae->setVelocities(d);
	    }
	    else cerr<<"create(ArticulatedSolidf*& obj, XML::Node<Core::DynamicModel>* arg): can not set dof values of a non-EulerXYZ solid"<<endl;
	}
	
	obj->setIntraLinkFrame( jointCenter, jointAxis, jointAngle );
	obj->setInertia( m, massCenter, xx,yy,zz,xy,yz,zx);
}

SOFA_DECL_CLASS(ArticulatedSolidf)

Creator< XML::Node<Core::DynamicModel>::Factory, ArticulatedSolidf > ArticulatedSolidfClass("ArticulatedSolidf");

}
}
}


