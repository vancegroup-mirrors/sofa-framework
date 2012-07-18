/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include "Binding_SofaModule.h"
#include "Binding_BaseObject.h"
#include "Binding_BaseState.h"
#include "Binding_Node.h"
#include "PythonMacros.h"

#include <sofa/core/ObjectFactory.h>
#include <sofa/gui/SofaGUI.h>
#include <sofa/gui/GUIManager.h>


using namespace sofa::core;
using namespace sofa::core::objectmodel;

#include <sofa/simulation/common/Node.h>
using namespace sofa::simulation;


// object factory
extern "C" PyObject * Sofa_createObject(PyObject * /*self*/, PyObject * /*args*/)
{
    /*
    PyObject* pyContext;
    PyObject* pyDesc;
    if (!PyArg_ParseTuple(args, "OO",&pyContext,&pyDesc))
        return 0;
    BaseContext *context=dynamic_cast<BaseContext*>(((PySPtr<Base>*)pyContext)->object.get());
    BaseObjectDescription *desc=(((PyPtr<BaseObjectDescription>*)pyDesc)->object);

    BaseObject::SPtr obj = ObjectFactory::getInstance()->createObject(context,desc);//.get();
    if (obj==0)
    {
        printf("<PYTHON> ERROR createObject '%s' of type '%s' in node '%s'\n",
                desc->getName().c_str(),
                desc->getAttribute("type",""),
                context->getName().c_str());
        PyErr_BadArgument();
        return 0;
    }

    obj->init();


    // par défaut, ce sera toujours au minimum un BaseObject...
    return SP_BUILD_PYSPTR(obj.get());
    */
    // deprecated on date 2012/07/18
    printf("<PYTHON> ERROR: Sofa.creatObject(BaseContext,BaseObjectDescription) is deprecated.\nPlease use BaseContext.createObject(BaseObjectDescription) instead.");
    PyErr_BadArgument();
    return 0;
}


extern "C" PyObject * Sofa_getObject(PyObject * /*self*/, PyObject * /*args*/)
{
    /*
        PyObject* pyContext;
        char *path;
        if (!PyArg_ParseTuple(args, "Os",&pyContext,&path))
            return 0;
        BaseContext *context=dynamic_cast<BaseContext*>(((PySPtr<Base>*)pyContext)->object.get());
        if (!context || !path)
        {
            PyErr_BadArgument();
            return 0;
        }
        BaseObject::SPtr sptr;
        context->get<BaseObject>(sptr,path);

        return SP_BUILD_PYSPTR(sptr.get());
    */
    // deprecated on date 2012/07/18
    printf("<PYTHON> ERROR: Sofa.getObject(BaseContext,path) is deprecated.\nPlease use BaseContext.getObject(path) instead.");
    PyErr_BadArgument();
    return 0;

}

extern "C" PyObject * Sofa_getChildNode(PyObject * /*self*/, PyObject * /*args*/)
{
    /*
    PyObject* pyBaseNode;
    char *path;
    if (!PyArg_ParseTuple(args, "Os",&pyBaseNode,&path))
        return 0;
    BaseNode *node=dynamic_cast<BaseNode*>(((PySPtr<Base>*)pyBaseNode)->object.get());
    if (!node || !path)
    {
        PyErr_BadArgument();
        return 0;
    }

    const objectmodel::BaseNode::Children& children = node->getChildren();
    Node *childNode = 0;
    // BaseNode ne pouvant pas être bindé en Python, et les BaseNodes des graphes étant toujours des Nodes,
    // on caste directement en Node.
    for (unsigned int i=0; i<children.size(); ++i)
        if (children[i]->getName() == path)
        {
            childNode = dynamic_cast<Node*>(children[i]);
            break;
        }
    if (!childNode)
    {
        printf("<PYTHON> Error: Sofa.getChildNode(%s) not found.\n",path);
        return 0;
    }
    return SP_BUILD_PYSPTR(childNode);
    */
    // deprecated on date 2012/07/18
    printf("<PYTHON> ERROR: Sofa.getChildNode(Node,path) is deprecated.\nPlease use Node.getChild(path) instead.");
    PyErr_BadArgument();
    return 0;
}

using namespace sofa::gui;

// send a text message to the GUI
extern "C" PyObject * Sofa_sendGUIMessage(PyObject * /*self*/, PyObject * args)
{
    char *msgType;
    char *msgValue;
    if (!PyArg_ParseTuple(args, "ss",&msgType,&msgValue))
        return 0;
    SofaGUI *gui = GUIManager::getGUI();
    if (!gui)
    {
        printf("<PYTHON> ERROR sendGUIMessage(%s,%s): no GUI !!\n",msgType,msgValue);
        return Py_BuildValue("i",-1);
    }
    gui->sendMessage(msgType,msgValue);


    return Py_BuildValue("i",0);
}




// Méthodes du module
SP_MODULE_METHODS_BEGIN(Sofa)
SP_MODULE_METHOD(Sofa,createObject)     // deprecated on date 2012/07/18
SP_MODULE_METHOD(Sofa,getObject)        // deprecated on date 2012/07/18
SP_MODULE_METHOD(Sofa,getChildNode)     // deprecated on date 2012/07/18
SP_MODULE_METHOD(Sofa,sendGUIMessage)
SP_MODULE_METHODS_END



