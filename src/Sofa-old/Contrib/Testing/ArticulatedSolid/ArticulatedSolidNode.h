//
// C++ Interface: ArticulatedSolidNode
//
// Description: 
//
//
// Author: The SOFA team </www.sofa-framework.org>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef Sofa_Components_XMLArticulatedSolidNode_h
#define Sofa_Components_XMLArticulatedSolidNode_h

#include <Sofa-old/Components/XML/Node.h>
//#include <Sofa-old/Abstract/BehaviorModel.h>
#include <Sofa-old/Core/DynamicModel.h>

namespace Sofa {

namespace Components {

namespace XML {

/**
XML node for an ArticulatedSolid.

@author The SOFA team
*/
class ArticulatedSolidNode : public Sofa::Components::XML::Node<Core::DynamicModel>
{
public:
  ArticulatedSolidNode(std::string& name, std::string& type);

    ~ArticulatedSolidNode();

    virtual const char* getClass() const;
	
    virtual bool setParent(BaseNode* parent);
    
    virtual bool initNode();
/*    virtual bool addChild(BaseNode* child);*/
};

}

}

}

#endif
