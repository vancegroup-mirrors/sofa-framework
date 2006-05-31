#ifndef SOFA_COMPONENTS_XML_NODE_H
#define SOFA_COMPONENTS_XML_NODE_H

#include "BaseNode.h"
#include "../Common/Factory.h"

namespace Sofa
{

namespace Components
{

namespace XML
{

using namespace Common;

template<class Object>
class Node : public BaseNode
{
private:
    Object* object;
public:
    Node(const std::string& name, const std::string& type, BaseNode* newParent=NULL)
        : BaseNode(name, type, newParent), object(NULL)
    {
    }

    virtual ~Node() {}

    Object* getObject()
    { return object; }

    virtual void setObject(Object* newObject)
    { object = newObject; }

    /// Get the associated object
    virtual Abstract::Base* getBaseObject() { return object; }

    virtual bool initNode();

    typedef Factory< std::string, Object, Node<Object>* > Factory;

};

template<class Object>
void createWithFilename(Object*& obj, BaseNode* arg)
{
    const char* filename = arg->getAttribute("filename");
    if (!filename)
    {
        std::cerr << arg->getType() << " requires a filename attribute\n";
        obj = NULL;
    }
    else
        obj = new Object(filename);
}

template<class Object, class ParentObject>
void createWithParentAndFilename(Object*& obj, BaseNode* arg)
{
    obj = NULL;
    const char* filename = arg->getAttribute("filename");
    if (!filename || arg->getParent()==NULL)
    {
        std::cerr << arg->getType() << " requires a filename attribute and a parent node\n";
        return;
    }
    ParentObject* object = dynamic_cast<ParentObject*>(arg->getParent()->getBaseObject());
    if (object==NULL)
    {
        // look for mechanicalmodel
        Abstract::BaseContext* ctx = dynamic_cast<Abstract::BaseContext*>(arg->getParent()->getBaseObject());
        if (ctx!=NULL)
            object = dynamic_cast<ParentObject*>(ctx->getMechanicalModel());
    }
    if (object==NULL) return;
    obj = new Object(object, filename);
}

template<class Object, class ParentObject>
void createWithParent(Object*& obj, BaseNode* arg)
{
    obj = NULL;
    ParentObject* object = dynamic_cast<ParentObject*>(arg->getParent()->getBaseObject());
    if (object==NULL)
    {
        // look for mechanicalmodel
        Abstract::BaseContext* ctx = dynamic_cast<Abstract::BaseContext*>(arg->getParent()->getBaseObject());
        if (ctx!=NULL)
            object = dynamic_cast<ParentObject*>(ctx->getMechanicalModel());
    }
    if (object==NULL) return;
    obj = new Object(object);
}

template<class Object, class Object1, class Object2>
void createWith2ObjectsAndFilename(Object*& obj, BaseNode* arg)
{
    obj = NULL;
    const char* filename = arg->getAttribute("filename");
    const char* object1 = arg->getAttribute("object1","../..");
    const char* object2 = arg->getAttribute("object2","..");
    if (!filename || !object1 || !object2)
    {
        std::cerr << arg->getType()<< " requires filename, object1 and object2 attributes\n";
        return;
    }
    Abstract::Base* pbase1 = arg->findObject(object1);
    Object1* pobject1 = dynamic_cast<Object1*>(pbase1);
    if (pobject1==NULL && pbase1!=NULL)
    {
        // look for mechanicalmodel
        Abstract::BaseContext* ctx = dynamic_cast<Abstract::BaseContext*>(pbase1);
        if (ctx!=NULL)
            pobject1 = dynamic_cast<Object1*>(ctx->getMechanicalModel());
    }
    Abstract::Base* pbase2 = arg->findObject(object2);
    Object2* pobject2 = dynamic_cast<Object2*>(pbase2);
    if (pobject2==NULL && pbase2!=NULL)
    {
        // look for mechanicalmodel
        Abstract::BaseContext* ctx = dynamic_cast<Abstract::BaseContext*>(pbase2);
        if (ctx!=NULL)
            pobject2 = dynamic_cast<Object2*>(ctx->getMechanicalModel());
    }
    if (pobject1==NULL || pobject2==NULL)
    {
        //std::cerr << arg->getType()<<": object1 "<<(pobject1?"OK":arg->findObject(object1)?"INVALID":"NULL")
        //                           <<", object2 "<<(pobject2?"OK":arg->findObject(object2)?"INVALID":"NULL")<<std::endl;
        return;
    }
    obj = new Object(pobject1, pobject2, filename);
}

template<class Object, class Object1, class Object2>
void createWith2Objects(Object*& obj, BaseNode* arg)
{
    obj = NULL;
    const char* object1 = arg->getAttribute("object1","../..");
    const char* object2 = arg->getAttribute("object2","..");
    if (!object1 || !object2)
    {
        std::cerr << arg->getType()<< " requires object1 and object2 attributes\n";
        return;
    }
    Abstract::Base* pbase1 = arg->findObject(object1);
    Object1* pobject1 = dynamic_cast<Object1*>(pbase1);
    if (pobject1==NULL && pbase1!=NULL)
    {
        // look for mechanicalmodel
        Abstract::BaseContext* ctx = dynamic_cast<Abstract::BaseContext*>(pbase1);
        if (ctx!=NULL)
            pobject1 = dynamic_cast<Object1*>(ctx->getMechanicalModel());
    }
    Abstract::Base* pbase2 = arg->findObject(object2);
    Object2* pobject2 = dynamic_cast<Object2*>(pbase2);
    if (pobject2==NULL && pbase2!=NULL)
    {
        // look for mechanicalmodel
        Abstract::BaseContext* ctx = dynamic_cast<Abstract::BaseContext*>(pbase2);
        if (ctx!=NULL)
            pobject2 = dynamic_cast<Object2*>(ctx->getMechanicalModel());
    }
    if (pobject1==NULL || pobject2==NULL)
    {
        //std::cerr << arg->getType()<<": object1 "<<(pobject1?"OK":arg->findObject(object1)?"INVALID":"NULL")
        //                           <<", object2 "<<(pobject2?"OK":arg->findObject(object2)?"INVALID":"NULL")<<std::endl;
        return;
    }
    obj = new Object(pobject1, pobject2);
}

} // namespace XML

} // namespace Components

} // namespace Sofa

#endif
