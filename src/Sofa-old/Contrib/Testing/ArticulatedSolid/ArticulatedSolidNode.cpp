//
// C++ Implementation: ArticulatedSolidNode
//
// Description:
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "ArticulatedSolidNode.h"
#include <Sofa-old/Components/XML/DynamicNode.h>
#include <Sofa-old/Components/XML/GroupNode.h>
#include <Sofa-old/Components/XML/SceneNode.h>
#include "ArticulatedSolid.h"
#include "ArticulatedSolidTypes.h"
#include <iostream>
using std::cout;
using std::endl;

namespace Sofa
{

namespace Components
{

  typedef ArticulatedSolid< ArticulatedSolidTypes<float> > ArticulatedSolidf;

namespace XML
{

ArticulatedSolidNode::ArticulatedSolidNode(std::string& name, std::string& type)
                : Sofa::Components::XML::Node<Core::DynamicModel>(name, type)
{
        cout<<"ArticulatedSolidNode::ArticulatedSolidNode, name="<<name<<", type="<<type<<endl;
}


ArticulatedSolidNode::~ArticulatedSolidNode()
{}

SOFA_DECL_CLASS(ArticulatedSolidNode)

Creator<DynamicNode::NodeFactory, ArticulatedSolidNode> ArticulatedSolidNodeClass("ArticulatedSolid");

const char* ArticulatedSolidNode::getClass() const
{
        return ArticulatedSolidNodeClass.c_str();
}

bool ArticulatedSolidNode::setParent(BaseNode* parent)
{
        cout<<"ArticulatedSolidNode::setParent"<<endl;
        if (dynamic_cast<ArticulatedSolidNode*>(parent)==NULL
                        && dynamic_cast<GroupNode*>(parent)==NULL)
                return false;
        else
                return Node<Core::DynamicModel>::setParent(parent);
}

bool ArticulatedSolidNode::initNode()
{
  if (!Node<Core::DynamicModel>::initNode()) return false;
  
  ArticulatedSolidf* articulatedSolid = dynamic_cast<ArticulatedSolidf*>( getObject() );
  assert( articulatedSolid );
  assert( getParent() );
  
  //Scene::getInstance()->addVisualModel( articulatedSolid );
	
  if (Scene* scene = dynamic_cast<Scene*>(getParent()->getBaseObject()) )
  {
    std::cout << "Adding DynamicModel "<<getName()<<" to Scene "<<getParent()->getName()<<std::endl;
    scene->addDynamicModel(getName(), getObject());
  }
  else if (Core::MechanicalGroup* group = dynamic_cast<Core::MechanicalGroup*>(getParent()->getBaseObject()) )
  {
    std::cout << "Adding DynamicModel "<<getName()<<" to Group "<<getParent()->getName()<<std::endl;
    group->addObject(getObject());
  }
  else if (ArticulatedSolidf* parentSolid = dynamic_cast<ArticulatedSolidf*>(getParent()->getBaseObject()) )
  {
    std::cout << "Adding DynamicModel "<<getName()<<" to DynamicObject "<<getParent()->getName()<<std::endl;
    parentSolid->addChild(articulatedSolid);
  }
  else
    std::cerr << "Orphan DynamicModel "<<getName()<<std::endl;
  return true;
}

/*bool ArticulatedSolidNode::addChild(BaseNode* child)
{
        cout<<"ArticulatedSolidNode::addChild"<<endl;
        return BaseNode::addChild(child);
}*/
}

}

}

